#!/usr/bin/env python3

import xml.etree.cElementTree as ET


import sys

tag_name_g = "{http://www.w3.org/2000/svg}g"
tag_name_rect = "{http://www.w3.org/2000/svg}rect"
tag_name_circle = "{http://www.w3.org/2000/svg}circle"
tag_name_path = "{http://www.w3.org/2000/svg}path"


def print_usage():
    print("Usage: svg_to_world.py <INPUT-SVG> [OUTPUT-WORLD]")


def get_translate(translate):
    translate = str(translate)
    index = translate.find("(")
    end = translate.find(",", index )
    x = float(translate[index + 1:end])
    index = end + 1
    end = translate.find(")", index)
    y = float(translate[index + 1:end])

    return x, y


def main():

    print("*** svg_to_world.py ***")

    svg_filename = ""
    world_filename = ""

    # Check input arguments
    if len(sys.argv) == 2 or len(sys.argv) == 3:
        svg_filename = str(sys.argv[1])
        if not svg_filename.endswith(".svg"):
            print_usage()
            print("The input file must be a svg file!")
            exit(0)

        if len(sys.argv) == 2:
            world_filename = svg_filename[:-4] + ".world"
        else:
            world_filename = str(sys.argv[2]) + ".world"

    else:
        print_usage()
        exit(0)

    tree = ET.parse(svg_filename)
    root = tree.getroot()

    # Get the group tag
    group = root.find(tag_name_g)

    translate_x, translate_y = get_translate(group.attrib["transform"])

    data_circle = []
    data_rect = []
    data_path = []

    print("-> Read svg Figures")

    for figure in group.iter():
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

            data_circle.append([x, y, r])

        elif figure.tag == tag_name_path:
            path_string = str(figure.attrib['d']).split(" ")

            path = []

            for point in path_string:
                point = point.split(",")
                if len(point) == 2:
                    path.append(float(point[0]))
                    path.append(float(point[1]))

            if len(path) != 0:
                data_path.append(path)

    print("-> Write world file")

    world_file = open(world_filename, "w")

    world_file.write("# lines -> x1 y1 x2 y2\n")

    for path in data_path:

        for i in range(0, len(path) - 2, 2):
            line_start_x = float(path[i]) + translate_x
            line_start_y = float(path[i + 1]) + translate_y
            line_end_x = float(path[i + 2]) + translate_x
            line_end_y = float(path[i + 3]) + translate_y

            line = str(line_start_x) + " " + str(line_start_y) + " " + str(line_end_x) + " " + str(line_end_y) + "\n"
            world_file.write(line)

    for rect in data_rect:
        x = float(rect[0]) + translate_x
        y = float(rect[1]) + translate_y
        width = float(rect[2])
        height = float(rect[3])

        line = str(x) + " " + str(y) + " " + str(x + width) + " " + str(y) + "\n"
        world_file.writelines(line)

        line = str(x + width) + " " + str(y) + " " + str(x + width) + " " + str(y + height) + "\n"
        world_file.writelines(line)

        line = str(x) + " " + str(y) + " " + str(x) + " " + str(y + height) + "\n"
        world_file.writelines(line)

        line = str(x) + " " + str(y + height) + " " + str(x) + " " + str(y + height) + "\n"
        world_file.writelines(line)

    world_file.write("# circles -> x y r\n")

    for circle in data_circle:
        x = float(circle[0]) + translate_x
        y = float(circle[1]) + translate_y
        r = float(circle[2])

        c = str(x) + " " + str(y) + " " + str(r) + "\n"
        world_file.write(c)

    print("DONE")


if __name__ == "__main__":
    main()