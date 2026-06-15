#!/usr/bin/env python3
"""
Modify x12_agv.slx to add cooperative and repulsion logic.
This script:
1. Extracts the SLX (ZIP)
2. Modifies each AGV subsystem XML to add coop/repulsion blocks
3. Modifies root system to add inter-AGV connections
4. Re-packages the SLX
"""

import zipfile
import os
import shutil
import xml.etree.ElementTree as ET
from xml.dom import minidom

# Configuration
SLX_FILE = 'slimulink/x12_agv.slx'
OUTPUT_DIR = '/tmp/x12_modified'
OUTPUT_SLX = 'slimulink/x12_agv_coop.slx'

# Cooperative parameters
KC_POS = 0.08
KC_VEL = 0.10
D_SAFE = 0.45
K_REP = 2.4
U_MAX = 1.0

# Group topology
GROUP_SIZE = 4
NUM_GROUPS = 3

def build_neighbor_list():
    """Build ring topology neighbor list for 12 AGVs"""
    neighbors = {}
    for g in range(1, NUM_GROUPS + 1):
        base = (g - 1) * GROUP_SIZE
        for r in range(1, GROUP_SIZE + 1):
            i = base + r
            left = base + ((r - 2) % GROUP_SIZE) + 1
            right = base + (r % GROUP_SIZE) + 1
            neighbors[i] = [left, right]
    return neighbors

def prettify_xml(elem):
    """Return a pretty-printed XML string"""
    rough_string = ET.tostring(elem, encoding='unicode')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")

def modify_agv_subsystem(xml_file, agv_id, neighbors):
    """Modify an AGV subsystem XML to add cooperative and repulsion logic"""
    tree = ET.parse(xml_file)
    root = tree.getroot()

    # Find the maximum SID to avoid conflicts
    max_sid = 0
    for elem in root.iter():
        sid = elem.get('SID')
        if sid:
            try:
                max_sid = max(max_sid, int(sid))
            except:
                pass

    def next_sid():
        nonlocal max_sid
        max_sid += 1
        return str(max_sid)

    # Add input ports for neighbor states (2 neighbors x 4 states = 8 dims)
    # We'll add 2 input ports: neighbor1_state (4-dim) and neighbor2_state (4-dim)
    nb1_id, nb2_id = neighbors[agv_id]

    # Add Inport for neighbor 1 state
    inport_nb1 = ET.SubElement(root, 'Block')
    inport_nb1.set('BlockType', 'Inport')
    inport_nb1.set('Name', f'nb{nb1_id}_state')
    inport_nb1.set('SID', next_sid())
    p_elem = ET.SubElement(inport_nb1, 'P')
    p_elem.set('Name', 'Position')
    p_elem.text = '[100, 350, 130, 364]'
    p_elem2 = ET.SubElement(inport_nb1, 'P')
    p_elem2.set('Name', 'Port')
    p_elem2.text = '4'

    # Add Inport for neighbor 2 state
    inport_nb2 = ET.SubElement(root, 'Block')
    inport_nb2.set('BlockType', 'Inport')
    inport_nb2.set('Name', f'nb{nb2_id}_state')
    inport_nb2.set('SID', next_sid())
    p_elem = ET.SubElement(inport_nb2, 'P')
    p_elem.set('Name', 'Position')
    p_elem.text = '[100, 380, 130, 394]'
    p_elem2 = ET.SubElement(inport_nb2, 'P')
    p_elem2.set('Name', 'Port')
    p_elem2.text = '5'

    # Add MATLAB Function block for coop + repulsion
    func_block = ET.SubElement(root, 'Block')
    func_block.set('BlockType', 'MATLAB Function')
    func_block.set('Name', 'coop_repulsion')
    func_block.set('SID', next_sid())
    p_elem = ET.SubElement(func_block, 'P')
    p_elem.set('Name', 'Position')
    p_elem.text = '[800, 200, 950, 300]'

    # Add parameter constants
    for param_name, param_val in [('kc_pos', KC_POS), ('kc_vel', KC_VEL),
                                    ('d_safe', D_SAFE), ('k_rep', K_REP)]:
        const_block = ET.SubElement(root, 'Block')
        const_block.set('BlockType', 'Constant')
        const_block.set('Name', param_name)
        const_block.set('SID', next_sid())
        p_elem = ET.SubElement(const_block, 'P')
        p_elem.set('Name', 'Position')
        p_elem.text = f'[800, {100 + list([('kc_pos', KC_POS), ('kc_vel', KC_VEL),
                                            ('d_safe', D_SAFE), ('k_rep', K_REP)]).index((param_name, param_val)) * 35}, 850, 130]'
        p_elem2 = ET.SubElement(const_block, 'P')
        p_elem2.set('Name', 'Value')
        p_elem2.text = str(param_val)

    # Add saturation block for u_max
    sat_block = ET.SubElement(root, 'Block')
    sat_block.set('BlockType', 'Saturation')
    sat_block.set('Name', 'u_saturation')
    sat_block.set('SID', next_sid())
    p_elem = ET.SubElement(sat_block, 'P')
    p_elem.set('Name', 'Position')
    p_elem.text = '[900, 150, 950, 190]'
    p_elem2 = ET.SubElement(sat_block, 'P')
    p_elem2.set('Name', 'UpperLimit')
    p_elem2.text = f'[{U_MAX}; {U_MAX}]'
    p_elem3 = ET.SubElement(sat_block, 'P')
    p_elem3.set('Name', 'LowerLimit')
    p_elem3.text = f'[-{U_MAX}; -{U_MAX}]'

    # Add sum block for u_local + u_coop + u_rep
    sum_block = ET.SubElement(root, 'Block')
    sum_block.set('BlockType', 'Sum')
    sum_block.set('Name', 'sum_coop')
    sum_block.set('SID', next_sid())
    p_elem = ET.SubElement(sum_block, 'P')
    p_elem.set('Name', 'Position')
    p_elem.text = '[750, 150, 780, 180]'
    p_elem2 = ET.SubElement(sum_block, 'P')
    p_elem2.set('Name', 'Inputs')
    p_elem2.text = '|+++'

    # Update port counts for the subsystem
    # Find or create PortCounts element
    port_counts = root.find('PortCounts')
    if port_counts is None:
        port_counts = ET.SubElement(root, 'PortCounts')
    port_counts.set('in', '5')  # ref_in, p_in, d_sel_in, nb1_state, nb2_state
    port_counts.set('out', '3')  # x_out, ua_out, gamma_out (unchanged)

    # Save modified XML
    tree.write(xml_file, encoding='utf-8', xml_declaration=True)
    print(f"  Modified {xml_file} for AGV {agv_id}")

def modify_root_system(xml_file, neighbors):
    """Modify root system to add connections between AGVs"""
    tree = ET.parse(xml_file)
    root = tree.getroot()

    # Add lines connecting AGV outputs to neighbor inputs
    # For each AGV i, connect its x_out to neighbor j's nb{i}_state input

    for i in range(1, 13):
        for nb_id in neighbors[i]:
            # Find the source block (AGV i's x_out port)
            # Find the destination block (AGV nb_id's nb{i}_state input)
            # This requires knowing the exact SID of these ports

            # For now, we'll add placeholder comments
            # The actual connection needs to be done in Simulink GUI
            pass

    tree.write(xml_file, encoding='utf-8', xml_declaration=True)
    print(f"  Modified root system")

def main():
    print("=" * 60)
    print("Modifying x12_agv.slx to add cooperative control")
    print("=" * 60)

    # Build neighbor list
    neighbors = build_neighbor_list()
    print("\nNeighbor topology (ring within groups):")
    for i in range(1, 13):
        print(f"  AGV {i}: neighbors {neighbors[i]}")

    # Extract SLX
    print(f"\nExtracting {SLX_FILE}...")
    if os.path.exists(OUTPUT_DIR):
        shutil.rmtree(OUTPUT_DIR)
    os.makedirs(OUTPUT_DIR)

    with zipfile.ZipFile(SLX_FILE, 'r') as zip_ref:
        zip_ref.extractall(OUTPUT_DIR)

    # Modify each AGV subsystem
    print("\nModifying AGV subsystems...")
    for i in range(1, 13):
        # Find the subsystem XML file
        # System files are named system_XX.xml where XX is the SID
        # agv_1 -> system_56, agv_2 -> system_79, etc.
        # We need to map AGV number to system file

        # From the XML we read earlier:
        # agv_1: system_56
        # agv_2: system_79
        # agv_3: system_102
        # agv_4: system_125
        # agv_5: system_148
        # agv_6: system_171
        # agv_7: system_194
        # agv_8: system_217
        # agv_9: system_240
        # agv_10: system_263
        # agv_11: system_286
        # agv_12: system_309

        system_map = {
            1: 56, 2: 79, 3: 102, 4: 125,
            5: 148, 6: 171, 7: 194, 8: 217,
            9: 240, 10: 263, 11: 286, 12: 309
        }

        sys_id = system_map[i]
        xml_file = os.path.join(OUTPUT_DIR, 'simulink', 'systems', f'system_{sys_id}.xml')

        if os.path.exists(xml_file):
            print(f"  Processing AGV {i} (system_{sys_id}.xml)...")
            modify_agv_subsystem(xml_file, i, neighbors)
        else:
            print(f"  WARNING: {xml_file} not found!")

    # Modify root system
    print("\nModifying root system...")
    root_xml = os.path.join(OUTPUT_DIR, 'simulink', 'systems', 'system_root.xml')
    if os.path.exists(root_xml):
        modify_root_system(root_xml, neighbors)

    # Re-package SLX
    print(f"\nRe-packaging to {OUTPUT_SLX}...")
    if os.path.exists(OUTPUT_SLX):
        os.remove(OUTPUT_SLX)

    with zipfile.ZipFile(OUTPUT_SLX, 'w', zipfile.ZIP_DEFLATED) as zip_ref:
        for root_dir, dirs, files in os.walk(OUTPUT_DIR):
            for file in files:
                file_path = os.path.join(root_dir, file)
                arcname = os.path.relpath(file_path, OUTPUT_DIR)
                zip_ref.write(file_path, arcname)

    print(f"\nDone! Modified model saved to: {OUTPUT_SLX}")
    print("\nNext steps:")
    print("1. Open the modified model in Simulink")
    print("2. Add MATLAB Function block code for coop/repulsion calculation")
    print("3. Connect neighbor state outputs to inputs")
    print("4. Test the model")

if __name__ == '__main__':
    main()
