# asuming the image is 200 by 200 pixels and the real world dimensions are 1 by 1 meters(it says so in the colab notebook from canvas):

def convertCoordinates(px, py, imgW=200, imgH=200):

    px = max(0, min(px, imgW))
    py = max(0, min(py, imgH))

    norm_x= px/imgW
    norm_y= py/imgH

    min_x= -0.5
    max_x= 0.5
    min_y= -0.5
    max_y= 0.5

    world_x= min_x + norm_x*(max_x-min_x)
    world_y= max_y - norm_y*(max_y-min_y)
    world_z=0.25
    targetXYZ=[world_x, world_y, world_z]
    return (targetXYZ)