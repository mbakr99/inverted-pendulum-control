import rospy
import yaml
import os
from datetime import datetime

class GainsSaver:
    def __init__(self, file_path=None):
        """
        Initialize the GainsSaver with an optional file path.
        If no path is provided, saves to ~/.ros/controller_gains/
        """
        if file_path is None:
            # Use default ROS home directory
            ros_home = os.path.expanduser('~/.ros/controller_gains/')
            os.makedirs(ros_home, exist_ok=True)
            self.file_path = ros_home
        else:
            self.file_path = file_path
            os.makedirs(os.path.dirname(file_path), exist_ok=True)

    def save_gains(self, individual, fitness, metadata=None):
        """
        Save controller gains to a YAML file with timestamp
        
        Args:
            individual: The best individual containing the gains
            fitness: Fitness value(s) of the individual
            metadata: Optional dictionary of additional information to save
        """
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'controller_gains_{timestamp}.yaml'
        
        # Create the data structure
        data = {
            'gains': list(individual),
            'fitness': list(fitness) if hasattr(fitness, '__iter__') else [fitness],
            'timestamp': timestamp
        }
        
        # Add any additional metadata
        if metadata:
            data.update(metadata)
            
        # Save to YAML file
        full_path = os.path.join(self.file_path, filename)
        with open(full_path, 'w') as f:
            yaml.safe_dump(data, f, default_flow_style=False)
            
        rospy.loginfo(f"Saved controller gains to {full_path}")
        return full_path

    def load_latest_gains(self):
        """
        Load the most recent gains file
        
        Returns:
            dict: The loaded gains data
            None: If no gains file is found
        """
        try:
            files = [f for f in os.listdir(self.file_path) 
                    if f.startswith('controller_gains_')]
            if not files:
                return None
                
            latest_file = max(files)
            with open(os.path.join(self.file_path, latest_file), 'r') as f:
                return yaml.safe_load(f)
                
        except Exception as e:
            rospy.logerr(f"Error loading gains: {e}")
            return None
        


if __name__ == "__main__":
    print("GainsSaver was found")