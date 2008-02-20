
/* Copyright (c) 2006-2008, Stefan Eilemann <eile@equalizergraphics.com> 
   All rights reserved. */

#ifndef EQ_PLY_H
#define EQ_PLY_H

#include <eq/eq.h>

#include "vertexBufferRoot.h"

typedef mesh::VertexBufferRoot    Model;

/** The Equalizer Polygonal Rendering Example. */
namespace eqPly
{
    class LocalInitData;

    class Application : public eq::Client
    {
    public:
        Application( const LocalInitData& initData );
        virtual ~Application() {}

        /** Run an eqPly instance. */
        int run();
        
    protected:
        /** @sa eq::Client::clientLoop. */
        virtual bool clientLoop();
        
    private:
        const LocalInitData& _initData;
    };

    enum LogTopics
    {
        LOG_STATS = eq::LOG_CUSTOM,      // 4096
        LOG_CULL  = eq::LOG_CUSTOM << 1  // 8192
    };
}

#endif // EQ_PLY_H

