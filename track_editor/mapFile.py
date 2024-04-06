from ruamel.yaml import YAML 
from ruamel.yaml.comments import CommentedMap as OrderedDict
from pathlib import Path
import numpy as np

from guiLogic import landmarkType

class My_Yaml_Dump:
	def __init__(self, f) -> None:
		self.f = f


	def write(self, s):
		self.f.write(self.__clean_yaml(s.decode('utf-8')))


	def __clean_yaml(self, yaml_file: str) -> str:
		yaml_file = yaml_file.replace("'", "")
		return yaml_file

def stringToLandmarkType(string):
  if(string == "blue"):
    return landmarkType.BLUE
  elif(string == "yellow"):
    return landmarkType.YELLOW
  elif(string == "small-orange"):
    return landmarkType.ORANGE
  elif(string == "big-orange"):
    return landmarkType.BIG_ORANGE
  elif(string == "timekeeping"):
    return landmarkType.TIMEKEEPING
  elif(string == "invisible"):
     return landmarkType.INVISIBLE

  return landmarkType.UNDEFINED

def landmarkTypeToString(type):
  if(type == landmarkType.BLUE):
    return "blue"
  elif(type == landmarkType.YELLOW):
    return "yellow"
  elif(type == landmarkType.ORANGE):
    return "small-orange"
  elif(type == landmarkType.BIG_ORANGE):
    return "big-orange"
  elif(type == landmarkType.TIMEKEEPING):
    return "timekeeping"
  elif(type == landmarkType.INVISIBLE):
    return "invisible"
  return "unknown"

def writeYaml(fileName, cones, leftLane, rightLane, timeKeeping, startPose, earthToTrack):
    path = Path(fileName)
    start_position = startPose[0]
    start_orientation = startPose[1]
    originGeodeticCoordinates = earthToTrack[0]
    originENURotation = earthToTrack[1]

    left = []
    right = []
    time_keeping = []
    unknown = []
    for c in cones:
      unknown.append({"position": f"[{c[0][0]}, {c[0][1]}, {c[0][2]}]", "class": landmarkTypeToString(c[1])})
    for c in leftLane:
      left.append({"position": f"[{c[0][0]}, {c[0][1]}, {c[0][2]}]", "class": landmarkTypeToString(c[1])})
    for c in rightLane:
      right.append({"position": f"[{c[0][0]}, {c[0][1]}, {c[0][2]}]", "class": landmarkTypeToString(c[1])})
    for c in timeKeeping:
      time_keeping.append({"position": f"[{c[0][0]}, {c[0][1]}, {c[0][2]}]", "class": landmarkTypeToString(c[1])})

    version_number = 0.9
    yaml_dict = OrderedDict({
      "version": str(version_number),
      "track": OrderedDict({
        "lanesFirstWithLastConnected" : True,
        "start": OrderedDict({
            "position": f'[{start_position[0]}, {start_position[1]}, {start_position[2]}]',
            "orientation": f'[{start_orientation[0]}, {start_orientation[1]}, {start_orientation[2]}]'}),
        "earthToTrack": OrderedDict({
            "position": f'[{originGeodeticCoordinates[0]}, {originGeodeticCoordinates[1]}, {originGeodeticCoordinates[2]}]',
            "orientation": f'[{originENURotation[0]}, {originENURotation[1]}, {originENURotation[2]}]'}),
        "left": left,
        "right": right,
        "time_keeping": time_keeping,
        "unknown": unknown
        })
      })
    with open(path, 'w+') as f:
      yaml_dumper = My_Yaml_Dump(f)
      yaml = YAML()
      yaml.dump(yaml_dict, yaml_dumper)
    return


def readYaml(fileName):
    path = Path(fileName)

    yaml=YAML(typ='safe')   # default, if not specfied, is 'rt' (round-trip)
    data = yaml.load(path)

    unkownCones = []
    leftCones = []
    rightCones = []
    timekeeping = []
    lanesFirstWithLastConnected = False
    startPose = [np.zeros(3), np.zeros(3)]
    earthToTrack = [np.zeros(3), np.zeros(3)]
    if('lanesFirstWithLastConnected') in data['track']:
      lanesFirstWithLastConnected = bool(data['track']['lanesFirstWithLastConnected'])
    if('start') in data['track']:
      startPose = [np.array(data['track']['start']['position']), np.array(data['track']['start']['orientation'])]
    if('earthToTrack') in data['track']:
      earthToTrack = [np.array(data['track']['earthToTrack']['position']), np.array(data['track']['earthToTrack']['orientation'])]
    for c in data['track']['left']:
        leftCones.append([np.array(c['position']), c['class']])
    for c in data['track']['right']:
        rightCones.append([np.array(c['position']), c['class']])
    for c in data['track']['time_keeping']:
        # overwrite class as timekeeping
        timekeeping.append([np.array(c['position']), landmarkTypeToString(landmarkType.TIMEKEEPING)])
    for c in data['track']['unknown']:
        unkownCones.append([np.array(c['position']), c['class']])
    return (unkownCones, leftCones, rightCones, timekeeping, lanesFirstWithLastConnected, startPose, earthToTrack)