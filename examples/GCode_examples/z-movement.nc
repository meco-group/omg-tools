(GCode used to generate a retract or engage movement of a tool in the z-axis

(engage: put gcode_block.py 'else: start at self.X0=-0.5 self.Y0=0'
G00 X-0.5 Y0
G01 X0 Y0

(retract
(G00 X0 Y0
(G01 X-0.5 Y0