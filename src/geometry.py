import rospy
def interpolate_rays(ray_ground, ray_up):
    assert ray_ground[2] < 0
    assert ray_up[2] >= 0

    t = -ray_ground[2] / (ray_up[2] - ray_ground[2])

    # Pick a ray 5% down from horizontal
    t *= 0.95

    return [
        t * ray_up[0] + (1-t) * ray_ground[0],
        t * ray_up[1] + (1-t) * ray_ground[1],
        t * ray_up[2] + (1-t) * ray_ground[2]
            ]

def intersect_camera_with_ground(camera_rays, camera_point):
    '''
    camera_rays should have the same z component in the camera frame, but
    should be passed in the map frame

    camera_point is the location of the camera in the map frame
    '''
    intersecting_rays = []
    for i in range(len(camera_rays)):
        ray = camera_rays[i]
        if ray[2] < 0:
            # This ray intersects the ground
            intersecting_rays.append(ray)
        else:
            if camera_rays[i-1][2] < 0:
                # Ray to the left intersects the ground
                intersecting_rays.append(
                    interpolate_rays(camera_rays[i-1], ray))
            if camera_rays[(i+1) % len(camera_rays)][2] < 0:
                # Ray to the right intersects the ground
                intersecting_rays.append(
                    interpolate_rays(camera_rays[(i+1) % len(camera_rays)], ray))

    return polygon_for_intersecting_rays(intersecting_rays, camera_point)

def polygon_for_intersecting_rays(rays, camera_point):
    polygon = []
    for ray in rays:
        scale = -camera_point[2] / ray[2]
        polygon.append([
            ray[0] * scale + camera_point[0],
            ray[1] * scale + camera_point[1],
            0.0
            ])
    return polygon
