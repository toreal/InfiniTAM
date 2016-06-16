import math

print('w/d:',640/480)

focallen=614
width=308
Rad2Deg = 180 / 3.14159
fov = math.atan(width / focallen ) * 2.0 * Rad2Deg
print('rgb camera')
print(fov)
ratio=width/234.326263
print('ratio',ratio)


focallen=474.874878
width=314.048645 
fov = math.atan(width / focallen ) * 2.0 * Rad2Deg
print('depth camera')
print(fov)
ratio=width/245.994995
print('ratio',ratio)


