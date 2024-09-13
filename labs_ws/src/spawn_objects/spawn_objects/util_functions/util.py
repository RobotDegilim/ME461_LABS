import os
import xacro 
from ament_index_python.packages import get_package_share_directory

def get_model_file(model):
        '''
            Retrieves the URDF form of a model
            
            Args: 
                model: name of the model as a string
            
            Returns: 
                string: URDF form of a model 
        '''
        
        model_file = ''
        
        #* Get Path to Package
        pkg_path = os.path.join(get_package_share_directory('sokoban'))
        model_dir = os.path.join(pkg_path,'models')
        model_name = model.lower() #* Input Sanitization
        
        if model_name == 'donut':
            model_file = os.path.join(model_dir, 'Donut', 'model.sdf')
        elif model_name == 'eight':
            model_file = os.path.join(model_dir, 'Infinity', 'model.sdf')
        elif model_name == 'square':
            model_file = os.path.join(model_dir, 'Kare', 'model.sdf')
        elif model_name == 'triangle':
            model_file = os.path.join(model_dir, 'b3gen', 'model.sdf')
        elif model_name == 'box':
            pkg_path = os.path.join(get_package_share_directory('sokoban'))
            xacro_file = os.path.join(pkg_path,'urdf','box.urdf.xacro')
            return xacro.process_file(xacro_file).toxml()
        else:
            print('Invalid model name')
            print('Resorting to Default Model')
            model_file = os.path.join(model_dir, 'Donut', 'model.sdf')
        
        #* Read File
        with open(model_file, 'r') as file:
                return file.read()
                