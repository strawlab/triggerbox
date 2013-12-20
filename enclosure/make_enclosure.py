from py2scad import *
import numpy as np

# this is based on py2scad examples/basic_enclosure.py

INCH2MM = 25.4

standoff_diam = 5.0
standoff_offset = 5.0

# Inside dimensions
component_height = 20
board_standoff_height = 3
top_margin = 10

z = component_height + board_standoff_height + top_margin

x,y,z = 150.0 + (standoff_diam+standoff_offset+1)*2, 62.0, z
board_x = 140 # width of board
print 'board_x',board_x
hole_list = []

BNC_HOLE = 13.0
#front_y0 = 20.0
front_y0 = z*0.5 - board_standoff_height

if 1:
    # Create BNC holes in front
    hole_right_x = x*0.5
    front_x0 = 0

    for hole_x in [ hole_right_x-4.0366*INCH2MM,
                    hole_right_x-3.2378*INCH2MM,
                    hole_right_x-2.3138*INCH2MM,
                    hole_right_x-1.3764*INCH2MM,
                    ]:
        hole_y = 12.0
        hole = {
            'panel'     : 'front',
            'type'      : 'round',
            'location'  : (hole_x-front_x0,hole_y-front_y0),
            'size'      : BNC_HOLE,
            }
        hole_list.append(hole)

if 1:
    # Create BNC holes in back (triggers)
    hole_right_x = x*0.5
    front_x0 = 0

    for hole_x in hole_right_x - np.array([1.2717,
                                           2.1417,
                                           3.0150,
                                           3.7909,
                                           4.5669,
                                           5.3614,
                                           ])*INCH2MM:
        hole_y = 12.0
        hole = {
            'panel'     : 'back',
            'type'      : 'round',
            'location'  : (hole_x-front_x0,hole_y-front_y0),
            'size'      : BNC_HOLE,
            }
        hole_list.append(hole)


if 1:
    # mini-USB plug
    usb_hole_x = hole_right_x-4.9642*INCH2MM
    usb_hole_y = hole_y + 5.1
    hole = {
        'panel'     : 'front',
        'type'      : 'square',
        'location'  : (usb_hole_x-front_x0,usb_hole_y-front_y0),
        'size'      : (.75*INCH2MM, .3*INCH2MM), # big enough for entire connector
        }
    hole_list.append(hole)

if 0:
    print '*'*80
    print 'WARNING - SHOWING PCB EDGES'
    print '*'*80

    # mark PCB edges
    for panel in 'front','back':
        for hole_x in [ hole_right_x - 0.6327*INCH2MM, hole_right_x - 6.1268*INCH2MM ]:
            for hole_y in np.linspace(0,y,20):
                hole = {
                    'panel'     : panel,
                    'type'      : 'round',
                    'location'  : (hole_x-front_x0,hole_y-front_y0),
                    'size'      : 1.5,
                    }
                hole_list.append(hole)


if 1:
    # mounting holes
    dd = 25.0

    nx = 8
    ny = 2
    mx0 = -nx*dd/2
    my0 = -ny*dd/2

    if 0:
        print '*'*2000
        print 'MOUNTING HOLE DIAMETER IS FOR TESTING DO NOT MAKE THIS PART'
        print '*'*2000
        hole_diam = 11.0
    else:
        hole_diam = 8.2

    for hole_x in [mx0, mx0+nx*dd]:
        for hole_y in [my0,my0+ny*dd]:
            hole = {
                'panel'     : 'bottom',
                'type'      : 'round',
                'location'  : (hole_x,hole_y),
                'size'      : hole_diam,
                }
            hole_list.append(hole)


overhang = 3.0
params = {
        'inner_dimensions'        : (x,y,z),
        'wall_thickness'          : 2.0,
        'lid_radius'              : 3.0,
        'top_x_overhang'          : overhang,
        'top_y_overhang'          : overhang,
        'bottom_x_overhang'       : 22.0,
        'bottom_y_overhang'       : overhang,
        'lid2front_tabs'          : (0.2,0.5,0.8),
        'lid2side_tabs'           : (0.25, 0.75),
        'side2side_tabs'          : (0.5,),
        'lid2front_tab_width'     : 0.75*INCH2MM,
        'lid2side_tab_width'      : 0.75*INCH2MM,
        'side2side_tab_width'     : 0.5*INCH2MM,
        'standoff_diameter'       : standoff_diam,
        'standoff_offset'         : standoff_offset,
        'standoff_hole_diameter'  : 3,
        'hole_list'               : hole_list,
        }

enclosure = Basic_Enclosure(params)
enclosure.make()

part_assembly = enclosure.get_assembly()
part_assembly_exploded = enclosure.get_assembly(explode=(5,5,5))
part_projection = enclosure.get_projection(show_ref_cube=False)

if 1:
    prog_assembly = SCAD_Prog()
    prog_assembly.fn = 50
    prog_assembly.add(part_assembly)
    prog_assembly.write('enclosure_assembly.scad')

if 1:
    prog_assembly_exploded = SCAD_Prog()
    prog_assembly_exploded.fn = 50
    prog_assembly_exploded.add(part_assembly_exploded)
    prog_assembly_exploded.write('enclosure_assembly_exploded.scad')

prog_projection = SCAD_Prog()
prog_projection.fn = 50
prog_projection.add(part_projection)
prog_projection.write('enclosure_projection.scad')
