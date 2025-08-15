find_package(OpenCV REQUIRED)

VisCore_add_exe(rune_detect_demo
DEPENDS core math contour_proc feature rune_target camera
EXTERNAL ${OpenCV_LIBS}
)