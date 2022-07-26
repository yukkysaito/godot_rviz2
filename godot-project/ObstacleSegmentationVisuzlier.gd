extends PointCloudVisualizer

func _ready():
	pass # Replace with function body.

func _init():
	subscribe("/perception/obstacle_segmentation/pointcloud", false)
	
func _process(_delta):
	visualize_latest_pointcloud()
	print(_delta)
