import numpy as np
import ground
import rigid_body

def collide(grnd, rb):
    corners = []
    normals = []
    depths = []
    if (rb.is_polygon):
        corner_pos = rb.get_corner_pos()
        for i in range(corner_pos.shape[0]):
            y_grnd = grnd.get_height(corner_pos[i,0])
            if (y_grnd > corner_pos[i,1]):
                corners.append(i)
                normals.append(grnd.get_normal(corner_pos[i,0]))
                depths.append((y_grnd - corner_pos[i,1])*normals[-1][1])
    else:
        ng = grnd.get_normal(rb.pos[0])
        cp = rb.pos - rb.radius*ng
        y_grnd = grnd.get_height(cp[0])
        if (y_grnd > cp[1]):
            normals.append(ng)
            v = cp - np.array([rb.pos[0], y_grnd])
            depths.append(v[0]*ng[0] + v[1]*ng[1])

    return corners, normals, depths
