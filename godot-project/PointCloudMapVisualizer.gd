extends PointCloudVisualizer


# Declare member variables here. Examples:
# var a = 2
# var b = "text"


# Called when the node enters the scene tree for the first time.
func _ready():
	pass # Replace with function body.

func _init():
	subscribe("/map/pointcloud_map", true)
	
func _process(_delta):
	visualize_latest_pointcloud()
