def convertCoordinates(px, py, ball_radius, min_x, max_x, min_y, max_y, imgW=200, imgH=200):

    px = max(0, min(px, imgW))
    py = max(0, min(py, imgH))

    norm_x= px/imgW
    norm_y= py/imgH

    world_x= min_x + norm_x*(max_x-min_x)
    world_y= max_y - norm_y*(max_y-min_y)
    targetXYZ=[world_x, world_y, ball_radius]
    return (targetXYZ)