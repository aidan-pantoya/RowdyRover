To run the Rover Blender Sim:
1. Download all files in /RowdyRover/Codes/* (After downloading ensure they are all in the same folder, will not work if they are not in the same folder)
2. Download Blender
3. cd into Blender directory to install imports:
(For me this is like this:
 a. 'cd ..'
 b. 'cd ..'
 c. 'cd Program Files'
 d. 'cd Blender Foundation'
 e. 'cd Blender 3.6'
 f. 'cd 3.6'
 g. 'cd python'
 h. 'cd bin'
 i. 'python.exe -m pip install opencv-python'
 j. 'python.exe -m pip install pillow'
 k. Install anything else needed this way ^^
 l. quit)

4. Launch Blender
5. File -> Open -> RowdyField.blend
6. script -> Open -> RoverSim.py
7. Change Output directory (line 14) to where ever you want to send your images.
8. Change Image range (line 176) to however long you wish to go.
9. Press run.
10. After run has completed, be sure to delete Rover_Body and cameras before running again.

NOTES:
1. CreateField.py is creating the field. If we use a pre built field, then we don't need this.
2. ImageStitch.py is just concatinating the 2 images. SLAM is needed.
3. PathFind.py is not being used. While it was good for real-life video, it is not good for blender (There is too much dirt for it)
4. RoverSim.py is creating the rover, cameras, taking images, finding the path, moving the rover, and continuing.
5. RowdyField.blend is a blank scene. Its purpose is so that Blender knows where the Python scripts are. It has to be in the same directory as the python scripts.
