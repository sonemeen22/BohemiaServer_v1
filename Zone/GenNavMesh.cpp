

#include <fstream>
#include <vector>
#include <cstdint>

bool LoadUnityGeometry(const char* path, InputMesh& out)
{
    std::ifstream in(path, std::ios::binary);
    if (!in) return false;

    int meshCount = 0;
    in.read((char*)&meshCount, sizeof(int));

    for (int m = 0; m < meshCount; ++m)
    {
        int vcount = 0;
        in.read((char*)&vcount, sizeof(int));

        int baseVertex = (int)(out.verts.size() / 3);

        for (int i = 0; i < vcount; ++i)
        {
            float x, y, z;
            in.read((char*)&x, sizeof(float));
            in.read((char*)&y, sizeof(float));
            in.read((char*)&z, sizeof(float));

            out.verts.push_back(x);
            out.verts.push_back(y);
            out.verts.push_back(z);
        }

        int icount = 0;
        in.read((char*)&icount, sizeof(int));

        for (int i = 0; i < icount; ++i)
        {
            int idx;
            in.read((char*)&idx, sizeof(int));
            out.tris.push_back(baseVertex + idx);
        }
    }

    return true;
}


