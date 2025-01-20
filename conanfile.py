from conan import ConanFile
from conan.tools.cmake import cmake_layout, CMake


class ProjectRecipe(ConanFile):
    settings = "os", "compiler", "build_type", "arch"
    generators = "CMakeToolchain", "CMakeDeps","vscode"
    name = 'delta-vins'
    version = '1.0.0'
    
    def requirements(self):
        self.requires("opencv/3.4.20")
        self.requires("eigen/3.4.0")
        self.requires("pangolin/0.9.1")

    def layout(self):
        cmake_layout(self)

    def build(self):
        # using cmake to build
        cmake = CMake(self)
        definitions = {}
        definitions['CMAKE_EXPORT_COMPILE_COMMANDS'] = 'ON'
        definitions['USE_ROS'] = 'OFF'
        cmake.configure(definitions)
        cmake.build()