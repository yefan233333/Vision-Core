
find_package(OpenCV REQUIRED)
RuneVision_add_module(core
EXTERNAL ${OpenCV_LIBS})
