import xml.etree.ElementTree as Et
import sys
from pathlib import Path
from ruamel.yaml import YAML 
# from collections import OrderedDict
from ruamel.yaml.comments import CommentedMap as OrderedDict

class My_Yaml_Dump:
	def __init__(self, f) -> None:
		self.f = f


	def write(self, s):
		self.f.write(self.__clean_yaml(s.decode('utf-8')))


	def __clean_yaml(self, yaml_file: str) -> str:
		yaml_file = yaml_file.replace("'", "")
		return yaml_file


def get_child_with_tag(root: Et.Element, tag: str) -> Et.Element:
	for child in root:
		if child.tag == tag:
			return child 
	return None


def get_children_with_tag(root: Et.Element, tag: str) -> list:
	result = []
	for child in root:
		if child.tag == tag:
			result.append(child)

	return result


def convert_to_object_class(uri: str) -> str:
	if uri == "cone_blue":
		return "blue"
	if uri == "cone_yellow":
		return "yellow"
	if uri == "time_keeping":
		return "timekeeping"
	if uri == "cone_orange":
		return "small-orange"
	if uri == "cone_orange_big":
		return "big-orange"
	return "unknown"


def extract_cones(elements: list) -> dict:
	left = []
	right = []
	time_keeping = []
	unknown = []

	for element in elements:
		pose = get_child_with_tag(element, "pose")
		url = get_child_with_tag(element, "uri")
		name = get_child_with_tag(element, "name")

		x = float(pose.text.rsplit(" ")[0])
		y = float(pose.text.rsplit(" ")[1])
		z = float(pose.text.rsplit(" ")[2])

		object_class = convert_to_object_class(str(url.text.rsplit("/")[-1]))

		if "cone_right" in name.text:
			right.append((x, y, z, object_class))
		elif "cone_left" in name.text:
			left.append((x, y, z, object_class))
		elif "tk_device" in name.text:
			time_keeping.append((x, y, z, object_class))
		else:
			unknown.append((x, y, z, object_class))

	return {"left": left, "right": right, "time_keeping": time_keeping, "unknown": unknown}


def convert_cones_to_yaml(cones: list) -> list:
	result = []
	for cone in cones:
		result.append({"position": f"[{cone[0]}, {cone[1]}, {cone[2]}]", "class": f"{cone[3]}"})

	return result


def write_to_yaml(cones: dict, file_path: str) -> None:
	left = convert_cones_to_yaml(cones["left"])
	right = convert_cones_to_yaml(cones["right"])
	time_keeping = convert_cones_to_yaml(cones["time_keeping"])
	unknown = convert_cones_to_yaml(cones["unknown"])
	start_position = (0.0, 0.0, 0.0)
	start_orientation = (0.0, 0.0, 0.0)

	yaml_file = OrderedDict({
		"track": OrderedDict({
		    "version": 1.0,
			"lanesFirstWithLastConnected": True,
			"start": OrderedDict({
					"position": f'[{start_position[0]}, {start_position[1]}, {start_position[2]}]',
					"orientation": f'[{start_orientation[0]}, {start_orientation[1]}, {start_orientation[2]}]'}),
      "earthToTrack": OrderedDict({
					"position": f'[{0.0}, {0.0}, {0.0}]',
					"orientation": f'[{0.0}, {0.0}, {0.0}]'}),
			"left": left,
			"right": right,
			"time_keeping": time_keeping,
			"unknown": unknown
			})
		})

	with open(Path(file_path).with_suffix('.yaml'), "w+") as f:
		f.write("# Map file for PacSim\n")
		yaml_dumper = My_Yaml_Dump(f)
		yaml = YAML()
		yaml.dump(yaml_file, yaml_dumper)


def main(file_path: str) -> None:
	tree = Et.parse(file_path)
	root = tree.getroot()
	model = get_child_with_tag(root, "model")
	objects = get_children_with_tag(model, "include")
	cones = extract_cones(objects)
	write_to_yaml(cones, file_path)


if __name__ == '__main__':
	if len(sys.argv) != 2:
		print("Call with python convert_sdf_to_yaml.py <file_path>")
	else:
		main(sys.argv[1])
