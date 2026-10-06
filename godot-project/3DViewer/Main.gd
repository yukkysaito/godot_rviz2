extends Node3D

# ROS is started by the RosBridge autoload (receiving on its own thread from app start-up).

func _ready():
	PerfMonitor.mark("main_ready")
