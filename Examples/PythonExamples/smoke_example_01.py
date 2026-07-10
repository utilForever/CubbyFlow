from pyCubbyFlow import *
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

ANIM_NUM_FRAMES = 360
ANIM_FPS = 60


def main():
    # Create smoke solver
    resX = 100
    solver = GridSmokeSolver2(resolution=(resX, 2 * resX), domainSizeX=1.0)

    # Customize pressure solver for real-time sim (less accurate, but much faster)
    pressureSolver = GridFractionalSinglePhasePressureSolver2()
    pressureSolver.linearSystemSolver = FDMGaussSeidelSolver2(20, 20, 0.001)
    solver.pressureSolver = pressureSolver

    # Setup emitter
    sphere = Sphere2(center=(0.5, 0.5), radius=0.15)
    emitter = VolumeGridEmitter2(sourceRegion=sphere)
    solver.emitter = emitter
    emitter.AddStepFunctionTarget(solver.smokeDensity, 0.0, 1.0)
    emitter.AddStepFunctionTarget(solver.temperature, 0.0, 1.0)

    # Visualization
    fig = plt.figure()
    den = np.array(solver.smokeDensity.GetDataAccessor(), copy=False)
    im = plt.imshow(
        den,
        vmin=0,
        vmax=1,
        cmap=plt.cm.gray,
        interpolation="nearest",
        animated=True,
        origin="lower",
    )

    # Animation
    frame = Frame(0, 1.0 / ANIM_FPS)

    def updateFig(*args):
        solver.Update(frame)
        frame.Advance()
        den = np.array(solver.smokeDensity.GetDataAccessor(), copy=False)
        im.set_data(den)
        return (im,)

    if len(sys.argv) > 1:
        output_format = sys.argv[1]
        if output_format == "gif":
            anim = animation.FuncAnimation(
                fig, updateFig, frames=ANIM_NUM_FRAMES, interval=ANIM_FPS, blit=True
            )
            anim.Save("SmokeExample1.gif", fps=ANIM_FPS, dpi=72, writer="imagemagick")
        elif output_format == "mp4":
            anim = animation.FuncAnimation(
                fig, updateFig, frames=ANIM_NUM_FRAMES, interval=ANIM_FPS, blit=True
            )
            anim.Save("SmokeExample1.mp4", fps=ANIM_FPS, writer="ffmpeg")
    else:
        animation.FuncAnimation(
            fig, updateFig, frames=ANIM_NUM_FRAMES, interval=1, blit=True
        )
        plt.show()


if __name__ == "__main__":
    Logging.Mute()
    main()
