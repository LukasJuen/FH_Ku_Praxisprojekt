#FH Kufstein Praxisprojekt - Wintersemester 2022
#SPS.bbM.21
#Michael Raffler und Lukas Juen

#main


import subprocess

script1 = "/home/Pi/Desktop/Test/FH_Ku_Praxisprojekt/Temp_cam.py"
script2 = "/home/Pi/Desktop/Test/FH_Ku_Praxisprojekt/PiLogger/PiLogger.py"

subprocess.Popen(["python", script1])
subprocess.Popen(["python", script2])


