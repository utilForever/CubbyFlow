/*************************************************************************
> File Name: Texture.h
> Project Name: CubbyFlow
> This code is based on Jet Framework that was created by Doyub Kim.
> References: https://github.com/doyubkim/fluid-engine-dev
> Purpose: the abstract base class for texture/
> Created Time: 2020/02/24
> Copyright (c) 2020, Ji-Hong snowapril
*************************************************************************/

#ifndef CUBBYFLOW_TEXTURE_H
#define CUBBYFLOW_TEXTURE_H

#include <Framework/Utils/Prerequisites.h>
#include <Framework/Utils/Common.h>
#include <cstdint>
#include <memory>

namespace CubbyFlow {
namespace CubbyRender {

    //! Texture sampling modes.
    enum class TextureSamplingMode : unsigned char
    {
        //! Sample nearest pixel.
        kNearest = 0,

        //! Linear-interpolate nearby pixels.
        kLinear = 1
    };

    //! Abstract base class for textures.
    class Texture
    {
    public:
        //! Default constructor.
        Texture();

        //! Destructor.
        virtual ~Texture();

        //! Returns the sampling mode of the texture.
        const TextureSamplingMode& getSamplingMode() const;

        //! Sets the sampling mode of the texture.
        void setSamplingMode(const TextureSamplingMode& mode);

        //! Bind texture to the shader in given slot id 
        void bind(RendererPtr renderer, unsigned int slotID);

        //! Destroy the texture from the renderer.
        void destroy();

     protected:
        //! Called when sampling mode has changed.
        virtual void onSamplingModeChanged(const TextureSamplingMode& mode) = 0;

        //! implementation of bind method
        virtual void onBind(RendererPtr renderer, unsigned int slotID) = 0;

        //! implementation of destry method
        virtual void onDestroy() = 0;        

        TextureSamplingMode _samplingMode = TextureSamplingMode::kNearest;
     private:
    };

    using TexturePtr = std::shared_ptr<Texture>;

} 
} 

#endif 