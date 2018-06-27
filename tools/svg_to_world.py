#!/usr/bin/env python3

import xml.etree.cElementTree as ET


import sys

tag_name_rect = "{http://www.w3.org/2000/svg}rect"
tag_name_circle = "{http://www.w3.org/2000/svg}circle"
tag_name_path = "{http://www.w3.org/2000/svg}path"
tag_name_group = "{http://www.w3.org/2000/svg}g"
tag_name_title = "{http://www.w3.org/2000/svg}title"


def print_usage():
    print("Usage: svg_to_world.py <INPUT-SVG> [SCALE]")


def get_border(view_box):
    view_box = str(view_box).split(" ")
    x = float(view_box[0])
    y = float(view_box[1])
    width = float(view_box[2])
    height = float(view_box[3])

    return x, y, width, height


def get_translate(group):
    if group is None:
        return 0, 0
    else:
        translate = group.attrib['transform']
        translate = str(translate)
        index = translate.find("(") + 1
        end = translate.find(",", index )
        x = float(translate[index:end])
        index = end + 1
        end = translate.find(")", index)
        y = -float(translate[index:end])
        return x, y


def get_node(title):
    title = title.split(" ")
    successful = True

    is_start = False
    is_end = False
    number = 0
    angle_start = 0
    angle_end = 0

    for value in title:
        value = value.split(":")

        if len(value) == 1:
            if value == ['s'] or value == ['S']:
                is_start = True
            elif value == ['e'] or value == ['E']:
                is_end = True
            elif not value == ['']:
                print("Unknown node in circle -> ignore this circle", value)
                successful = False
        elif len(value) == 2:
            if str(value[0]).lower() == 'id':
                number = int(value[1])
            elif str(value[0]).lower() == 'a':
                value = value[1].split(",")
                if len(value) == 1:
                    angle_start = float(value[0])
                    angle_end = angle_start
                elif len(value) == 2:
                    angle_start = float(value[0])
                    angle_end = float(value[1])
                else:
                    print("Unknown node in circle -> ignore the circle", value, len(value))
                    successful = False
            else:
                print("Unknown node in circle -> ignore this circle", value)
                successful = False
        else:
            print("Unknown node in circle -> ignore this circle", value)
            successful = False

    if not (is_start or is_end):
        successful = False

    return successful, is_start, is_end, number, angle_start, angle_end


def check_node(data_node):
    counter_start = 0
    counter_end = 0
    counter_start_end = 0

    for node in data_node:
        # x, y, r, is_start, is_end, number, angle_start, angle_end
        has_start = node[3]
        has_end = node[4]

        if has_start and has_end:
            counter_start_end += 1
        elif has_start:
            counter_start += 1
        elif has_end:
            counter_end += 1

    if counter_start + counter_start_end < 1 or counter_end + counter_start_end < 1:
        print("Error: Need at least one start and end point!")
        return False
    if counter_start < 1 and counter_end < 1:
        print("Error; Only one start_end point")
        return False
    return True





def main():

    print("*** svg_to_world.py ***")

    svg_filename = ""

    scale = 1

    # Check input arguments
    if len(sys.argv) == 2 or len(sys.argv) == 3:
        svg_filename = str(sys.argv[1])
        if not svg_filename.endswith(".svg"):
            print_usage()
            print("The input file must be a svg file!")
            exit(0)

        if len(sys.argv) == 3:
            scale = float(sys.argv[2])
    else:
        print_usage()
        exit(0)

    world_filename = svg_filename[:-4] + ".world"
    node_filename = svg_filename[:-4] + ".node"

    tree = ET.parse(svg_filename)
    root = tree.getroot()

    translate_x, translate_y = get_translate(root.find(tag_name_group))

    border_x, border_y, border_width, border_height = get_border(root.attrib["viewBox"])

    data_circle = []
    data_rect = []
    data_path = []
    data_node = []

    print("-> Read svg Figures")

    for figure in root.iter():
        if figure.tag == tag_name_rect:
            x = float(figure.attrib['x'])
            y = float(figure.attrib['y'])
            height = float(figure.attrib['height'])
            width = float(figure.attrib['width'])

            data_rect.append([x, y, width, height])

        elif figure.tag == tag_name_circle:
            r = float(figure.attrib['r'])
            x = float(figure.attrib['cx'])
            y = float(figure.attrib['cy'])

            title = figure.find(tag_name_title)

            if title is None:
                data_circle.append([x, y, r])
            else:
                successful, is_start, is_end, number, angle_start, angle_end = get_node(title.text)

                if successful:
                    data_node.append([x, y, r, is_start, is_end, number, angle_start, angle_end])

        elif figure.tag == tag_name_path:
            path_string = str(figure.attrib['d']).split(" ")

            is_successful = True

            is_absolute = True
            is_start_point = True

            path = []

            for value in path_string:
                value = value.split(",")

                if len(value) == 1:
                    if value == ['m']:
                        is_absolute = False
                        is_start_point = True
                    elif value == ['l']:
                        is_absolute = False
                        is_start_point = False
                    elif value == ['M'] or value == ['L']:
                        is_absolute = True
                        is_start_point = True
                    elif value == ['z'] or value == ['Z']:
                        path.append(path[0])
                        path.append(path[1])
                    else:
                        print("Unsupported option in path -> ignore this path", value)
                        is_successful = False
                        break
                elif len(value) == 2:
                    if is_start_point or is_absolute:
                        x = float(value[0])
                        y = float(value[1])

                        path.append(x)
                        path.append(y)

                        is_start_point = False
                    else:
                        x = path[-2] + float(value[0])
                        y = path[-1] + float(value[1])

                        path.append(x)
                        path.append(y)
            if is_successful:
                data_path.append(path)

    print("-> Check node Data")

    if not check_node(data_node):
        print("Error by checking node data!")
        exit(1)

    print("-> Write world file")

    world_file = open(world_filename, "w")

    world_file.write("# border lines -> x1 y1 x2 y2\n")

    border_scaled_x = border_x * scale
    border_scaled_y = border_y * scale
    border_scaled_width = border_width * scale
    border_scaled_height = border_height * scale

    border_line = str(border_scaled_x) + " " + str(border_scaled_y) + " " + str(border_scaled_x + border_scaled_width) + " " + str(border_scaled_y) + "\n"
    world_file.writelines(border_line)

    border_line = str(border_scaled_x) + " " + str(border_scaled_y) + " " + str(border_scaled_x) + " " + str(border_scaled_y + border_scaled_height) + "\n"
    world_file.writelines(border_line)

    border_line = str(border_scaled_x + border_scaled_width) + " " + str(border_scaled_y) + " " + str(border_scaled_x + border_scaled_width) + " " + str(border_scaled_y + border_scaled_height) + "\n"
    world_file.writelines(border_line)

    border_line = str(border_scaled_x) + " " + str(border_scaled_y + border_scaled_height) + " " + str(border_scaled_x + border_scaled_width) + " " + str(border_scaled_y + border_scaled_height) + "\n\n"
    world_file.writelines(border_line)

    world_file.write("# path lines -> x1 y1 x2 y2\n")

    for path in data_path:

        for i in range(0, len(path) - 2, 2):
            line_start_x = (float(path[i]) + translate_x) * scale
            line_start_y = (border_height - float(path[i + 1]) + translate_y) * scale
            line_end_x = (float(path[i + 2]) + translate_x) * scale
            line_end_y = (border_height - float(path[i + 3]) + translate_y) * scale

            line = str(line_start_x) + " " + str(line_start_y) + " " + str(line_end_x) + " " + str(line_end_y) + "\n"
            world_file.write(line)
        
        world_file.write("\n")

    world_file.write("# rectangle lines -> x1 y1 x2 y2\n")

    for rect in data_rect:
        x = (float(rect[0]) + translate_x) * scale
        y = (border_height - float(rect[1]) + translate_y) * scale
        width = (float(rect[2])) * scale
        height = (- float(rect[3])) * scale

        line = str(x) + " " + str(y) + " " + str(x + width) + " " + str(y) + "\n"
        world_file.writelines(line)

        line = str(x) + " " + str(y) + " " + str(x) + " " + str(y + height) + "\n"
        world_file.writelines(line)

        line = str(x + width) + " " + str(y) + " " + str(x + width) + " " + str(y + height) + "\n"
        world_file.writelines(line)

        line = str(x) + " " + str(y + height) + " " + str(x + width) + " " + str(y + height) + "\n\n"
        world_file.writelines(line)

    world_file.write("# circles -> x y r\n")

    for circle in data_circle:
        x = (float(circle[0]) + translate_x) * scale
        y = (border_height - float(circle[1]) + translate_y) * scale
        r = (float(circle[2])) * scale

        c = str(x) + " " + str(y) + " " + str(r) + "\n"
        world_file.write(c)

    print("-> Write node file")

    node_file = open(node_filename, "w")

    node_file.write("# x y r is_start is_end id angle_start angle_end\n")

    for node in data_node:
        x = (float(node[0]) + translate_x) * scale
        y = (border_height - float(node[1]) + translate_y) *  scale
        r = (float(node[2])) * scale
        is_start = node[3]
        is_end = node[4]
        number = int(node[5])
        angle_start = float(node[6])
        angle_end = float(node[7])

        node_file.write(str(x) + " " + str(y) + " " + str(r) + " " + str(is_start) + " " + str(is_end) + " "
                          + str(number) + " " + str(angle_start) + " " + str(angle_end) + "\n")

    print("DONE")


if __name__ == "__main__":
    main()