import os

if __name__ == "__main__":
	directory = "~/data_road/training/results_perspective"
	for filename in os.listdir(directory):
		if filename.startswith("um_"):
			os.rename(directory + "/" + filename, "um_road_" + filename[3:])
		if filename.startswith("uu_"):
			os.rename(directory + "/" + filename, "uu_road_" + filename[3:])
		if filename.startswith("umm_"):
			os.rename(directory + "/" + filename, "umm_road_" + filename[4:])
