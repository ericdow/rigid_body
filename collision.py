import ground
import rigid_body

def collide(grnd, rb):
    corners = []
    normals = []
    depths = []
    corner_pos = rb.get_corner_pos()
    for i in range(corner_pos.shape[0]):
        y_grnd = grnd.get_height(corner_pos[i,0])
        if (y_grnd > corner_pos[i,1]):
            corners.append(i)
            normals.append(grnd.get_normal(corner_pos[i,0]))
            depths.append((y_grnd - corner_pos[i,1])*normals[-1][1])

    return corners, normals, depths
