from pyCubbyFlow import *
import numpy as np
import os

ANIM_NUM_FRAMES = 360
ANIM_FPS = 60


def main():
    # Create APIC solver
    resX = 100
    solver = APICSolver3(resolution=(resX, resX, resX), domainSizeX=1.0)
    solver.useCompressedLinearSystem = True

    # Setup emitter
    bunny_filename = os.path.join(
        os.path.dirname(os.path.realpath(__file__)), "../../Resources/bunny.obj"
    )
    bunny_mesh = TriangleMesh3()
    bunny_mesh.ReadObj(bunny_filename)
    bunny_sdf = ImplicitTriangleMesh3(mesh=bunny_mesh, resolutionX=64, margin=0.2)
    emitter = VolumeParticleEmitter3(
        implicitSurface=bunny_sdf, spacing=1.0 / (2 * resX), isOneShot=False
    )
    solver.particleEmitter = emitter

    # Convert to surface
    grid_size = 1.0 / resX
    grid = VertexCenteredScalarGrid3(
        (resX, resX, resX), (grid_size, grid_size, grid_size)
    )

    def write_surface(frame_cnt, pos):
        converter = SPHPointsToImplicit3(1.5 * grid_size, 0.5)
        converter.Convert(pos.tolist(), grid)
        surface_mesh = MarchingCubes(
            grid, (grid_size, grid_size, grid_size), (0, 0, 0), 0.0, DIRECTION_ALL
        )
        surface_mesh.WriteObj("frame_{:06d}.obj".format(frame_cnt))

    # Make first frame
    frame = Frame(0, 1.0 / ANIM_FPS)

    for i in range(ANIM_NUM_FRAMES):
        print("Frame {:d}".format(i))
        solver.Update(frame)
        pos = np.array(solver.particleSystemData.positions, copy=False)
        write_surface(i, pos)
        frame.Advance()


if __name__ == "__main__":
    Logging.Mute()
    main()
