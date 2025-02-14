

#pragma once

//------------------------------------------------------------------------------
// リミッタ(共通関数)
template<typename T, typename U>
T limitter(const T& in, const U& min, const U& max)
{
	T wmax = static_cast<T>(max);
	T wmin = static_cast<T>(min);

    if(in < wmin)
    {
        return wmin;
    }
    if(wmax < in)
    {
        return wmax;
    }

    return in;
}

// End Of File -----------------------------------------------------------------
