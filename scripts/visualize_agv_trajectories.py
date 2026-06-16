"""
visualize_agv_trajectories.py
12-AGV trajectory visualization for 3-group cooperative control.
Reads step9_multi12_coop_demo.mat and plots:
  - Initial positions (stars)
  - Target positions (squares with formation)
  - Trajectories (colored lines, one per AGV)
  - Final positions (circles)
  - Group boundaries and formation offsets
"""
import numpy as np
import scipy.io as sio
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import os

# ── Load data ──────────────────────────────────────────────────
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
mat_path = os.path.join(project_root, 'step9_multi12_coop_demo.mat')
S = sio.loadmat(mat_path, squeeze_me=True)

x = S['x']           # (48, steps+1)  [px py vx vy] x 12 AGVs
x_ref = S['x_ref']   # (48,)
Ts = float(S['Ts'])
p = float(S['p'])
d = int(S['d'])

steps = x.shape[1] - 1
t = np.arange(steps + 1) * Ts
N = 12

# ── Extract positions ──────────────────────────────────────────
pos_all = np.zeros((N, 2, steps + 1))   # (agv, xy, time)
ref_all = np.zeros((N, 2))

for i in range(N):
    ix = slice(i * 4, i * 4 + 4)
    pos_all[i, 0, :] = x[ix, :][0, :]   # px
    pos_all[i, 1, :] = x[ix, :][1, :]   # py
    ref_all[i, 0] = x_ref[i * 4]
    ref_all[i, 1] = x_ref[i * 4 + 1]

# ── Group assignment ───────────────────────────────────────────
group_size = 4
num_groups = 3
group_colors = ['#e41a1c', '#377eb8', '#4daf4a']   # red, blue, green
group_labels = ['Group 1 (front fuselage)',
                'Group 2 (middle fuselage)',
                'Group 3 (rear fuselage)']
group_centers = np.array([[-1.5, 0.0], [0.0, 0.0], [1.5, 0.0]])

# ── Plot ───────────────────────────────────────────────────────
fig, ax = plt.subplots(1, 1, figsize=(16, 10))

# Draw group target regions
for g in range(num_groups):
    cx, cy = group_centers[g]
    # formation bounding box (0.5 x 0.5 square + margin)
    rect = patches.FancyBboxPatch(
        (cx - 0.45, cy - 0.45), 0.9, 0.9,
        boxstyle="round,pad=0.05",
        linewidth=1.5, linestyle='--',
        edgecolor=group_colors[g], facecolor=group_colors[g], alpha=0.06,
        label='_nolegend_'
    )
    ax.add_patch(rect)

# Plot trajectories
for i in range(N):
    g = i // group_size
    px = pos_all[i, 0, :]
    py = pos_all[i, 1, :]

    # Trajectory line (fading alpha: early=light, late=dark)
    # Draw as segments with increasing alpha
    n_seg = 20
    seg_len = max(1, len(px) // n_seg)
    for s in range(n_seg):
        i0 = s * seg_len
        i1 = min((s + 1) * seg_len + 1, len(px))
        alpha = 0.15 + 0.7 * (s / n_seg)
        lw = 0.8 + 1.0 * (s / n_seg)
        lbl = group_labels[g] if (i == g * group_size and s == n_seg - 1) else '_nolegend_'
        ax.plot(px[i0:i1], py[i0:i1],
                color=group_colors[g], alpha=alpha, linewidth=lw,
                label=lbl, zorder=2)

# Plot initial positions (stars)
for i in range(N):
    g = i // group_size
    ax.plot(pos_all[i, 0, 0], pos_all[i, 1, 0],
            marker='*', markersize=10, color=group_colors[g],
            markeredgecolor='black', markeredgewidth=0.5,
            zorder=5, label='_nolegend_')
    # label AGV index
    ax.annotate(f'{i+1}',
                (pos_all[i, 0, 0], pos_all[i, 1, 0]),
                textcoords='offset points', xytext=(5, 8),
                fontsize=7, color=group_colors[g], fontweight='bold')

# Plot final positions (filled circles)
for i in range(N):
    g = i // group_size
    ax.plot(pos_all[i, 0, -1], pos_all[i, 1, -1],
            marker='o', markersize=8, color=group_colors[g],
            markeredgecolor='black', markeredgewidth=0.8,
            fillstyle='full', zorder=5)

# Plot target positions (squares)
for i in range(N):
    g = i // group_size
    ax.plot(ref_all[i, 0], ref_all[i, 1],
            marker='s', markersize=9, color='white',
            markeredgecolor=group_colors[g], markeredgewidth=2.0,
            zorder=4)

# Draw arrows from initial to final for a few representative AGVs
for i in [0, 4, 8]:
    g = i // group_size
    ax.annotate('',
                xy=(pos_all[i, 0, -1], pos_all[i, 1, -1]),
                xytext=(pos_all[i, 0, 0], pos_all[i, 1, 0]),
                arrowprops=dict(arrowstyle='->', color=group_colors[g],
                                lw=1.2, alpha=0.4, linestyle='--'))

# Legend items (manual)
legend_elements = []
for g in range(num_groups):
    legend_elements.append(
        plt.Line2D([0], [0], color=group_colors[g], lw=2,
                   label=group_labels[g])
    )
legend_elements.append(
    plt.Line2D([0], [0], marker='*', color='w', markerfacecolor='gray',
               markersize=10, markeredgecolor='black', label='Initial position')
)
legend_elements.append(
    plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='gray',
               markersize=8, markeredgecolor='black', label='Final position')
)
legend_elements.append(
    plt.Line2D([0], [0], marker='s', color='w', markerfacecolor='white',
               markersize=9, markeredgecolor='gray', markeredgewidth=2,
               label='Target position')
)

ax.legend(handles=legend_elements, loc='upper right', fontsize=9,
          framealpha=0.9, edgecolor='gray')

# Formatting
ax.set_xlabel('X position (m)', fontsize=12)
ax.set_ylabel('Y position (m)', fontsize=12)
ax.set_title(
    f'12-AGV Cooperative Control Trajectories (3 Groups × 4 AGVs)\n'
    f'p={p:.2f}, d={d}, Ts={Ts}s, {steps} steps ({t[-1]:.1f}s total)',
    fontsize=13, fontweight='bold'
)
ax.set_aspect('equal')
ax.grid(True, alpha=0.3)

# Add group center labels
for g in range(num_groups):
    cx, cy = group_centers[g]
    ax.annotate(f'G{g+1} center\n({cx},{cy})',
                (cx, cy - 0.55),
                fontsize=8, color=group_colors[g], fontweight='bold',
                ha='center', va='top',
                bbox=dict(boxstyle='round,pad=0.2', facecolor='white',
                          edgecolor=group_colors[g], alpha=0.8))

# Add time markers on trajectories (every ~5s = 100 steps)
marker_interval = int(5.0 / Ts)
for i in [0, 4, 8]:
    g = i // group_size
    for k in range(marker_interval, steps, marker_interval):
        if k < steps + 1:
            ax.plot(pos_all[i, 0, k], pos_all[i, 1, k],
                    '.', markersize=3, color=group_colors[g], alpha=0.5)

plt.tight_layout()

# Save
pic_dir = os.path.join(project_root, 'pic')
os.makedirs(pic_dir, exist_ok=True)
out_path = os.path.join(pic_dir, 'agv12_trajectory_map.png')
plt.savefig(out_path, dpi=220, bbox_inches='tight')
print(f'Saved: {out_path}')

# ── Summary statistics ─────────────────────────────────────────
print('\n=== Trajectory Summary ===')
for g in range(num_groups):
    ids = range(g * group_size, (g + 1) * group_size)
    print(f'\nGroup {g+1} (center={group_centers[g]}):')
    for i in ids:
        init = pos_all[i, :, 0]
        final = pos_all[i, :, -1]
        ref = ref_all[i, :]
        err_final = np.linalg.norm(final - ref)
        dist_traveled = np.sum(np.sqrt(np.diff(pos_all[i, 0, :])**2 +
                                        np.diff(pos_all[i, 1, :])**2))
        print(f'  AGV{i+1:2d}: start=({init[0]:+.2f},{init[1]:+.2f}) -> '
              f'final=({final[0]:+.3f},{final[1]:+.3f})  '
              f'target=({ref[0]:+.2f},{ref[1]:+.2f})  '
              f'err={err_final:.4f}  dist={dist_traveled:.2f}m')

# Min pairwise distance over entire trajectory
min_d = np.inf
min_d_step = 0
for k in range(steps + 1):
    for i in range(N - 1):
        for j in range(i + 1, N):
            dij = np.linalg.norm(pos_all[i, :, k] - pos_all[j, :, k])
            if dij < min_d:
                min_d = dij
                min_d_step = k
print(f'\nMin pairwise distance: {min_d:.4f}m at t={min_d_step * Ts:.2f}s')

# Final group center errors
print('\n=== Group Center Tracking ===')
for g in range(num_groups):
    ids = range(g * group_size, (g + 1) * group_size)
    center_final = np.mean([pos_all[i, :, -1] for i in ids], axis=0)
    center_err = np.linalg.norm(center_final - group_centers[g])
    print(f'  Group {g+1}: center=({center_final[0]:+.4f},{center_final[1]:+.4f})  '
          f'target=({group_centers[g][0]:+.2f},{group_centers[g][1]:+.2f})  '
          f'err={center_err:.4f}')
