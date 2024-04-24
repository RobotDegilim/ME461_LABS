import os 
from ament_index_python.packages import get_package_share_directory

class Target:
    def __init__(self, name, x,y,z):
        self.name = name
        self.x = x
        self.y = y
        self.z = z
        self.model_file = ''
        self.pkg_path = os.path.join(get_package_share_directory('sokoban'))
        self.model_dir = os.path.join(self.pkg_path,'models')

    def get_model_file(self):
        if self.name == 'Donut':
            self.model_file = os.path.join(self.model_dir, 'Donut', 'model.sdf')
        elif self.name == 'Infinity':
            self.model_file = os.path.join(self.model_dir, 'Infinity', 'model.sdf')
        elif self.name == 'Kare':
            self.model_file = os.path.join(self.model_dir, 'Kare', 'model.sdf')
        elif self.name == 'b3gen':
            self.model_file = os.path.join(self.model_dir, 'b3gen', 'model.sdf')
        else:
            print('Invalid model name')
            # default model
            print('Resorting to Default Model')
            self.model_file = os.path.join(self.model_dir, 'Donut', 'model.sdf')
        return self.model_file

