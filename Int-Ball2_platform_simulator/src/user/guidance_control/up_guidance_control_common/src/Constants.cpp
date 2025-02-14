
#include "guidance_control_common/Constants.h"
#include "guidance_control_common/FileReader.h"
#include "guidance_control_common/Log.h"

#include <sstream>

//------------------------------------------------------------------------------
// デフォルトコンストラクタ
ib2_mss::Constants::Constants() :
re_(RE_GRS80), fe_(FE_GRS80), ue_(GME_JGM3), we_(WE_IERS2000), rho0_(RHO0)
{
}

//------------------------------------------------------------------------------
// デストラクタ
ib2_mss::Constants::~Constants() = default;

//------------------------------------------------------------------------------
// ログの構成
void ib2_mss::Constants::configure(const std::string& filename)
{
	static const std::string DEFAULT("DEFAULT");
	size_t lineno(0);
	try
	{
		FileReader f(filename);
		std::string re(f.string(lineno++));
		if (re == DEFAULT)
			instance().re_ = RE_GRS80;
		else
			instance().re_ = std::stod(re);
		std::string fe(f.string(lineno++));
		if (fe == DEFAULT)
			instance().fe_ = FE_GRS80;
		else
			instance().fe_ = std::stod(fe);
		std::string ue(f.string(lineno++));
		if (ue == DEFAULT)
			instance().ue_ = GME_JGM3;
		else
			instance().ue_ = std::stod(ue);
		std::string we(f.string(lineno++));
		if (we == DEFAULT)
			instance().we_ = WE_IERS2000;
		else
			instance().we_ = std::stod(we);
		std::string rho(f.string(lineno++));
		if (rho == DEFAULT)
			instance().rho0_ = RHO0;
		else
			instance().rho0_ = std::stod(rho);
	}
	catch (const std::exception& e)
	{
		LOG_ERROR(Log::errorFileLine(e.what(), "", filename, lineno));
		throw;
	}
}

//------------------------------------------------------------------------------
// 地球赤道半径の取得
double ib2_mss::Constants::re()
{
	return instance().re_;
}

//------------------------------------------------------------------------------
// 地球扁平率の取得
double ib2_mss::Constants::fe()
{
	return instance().fe_;
}

//------------------------------------------------------------------------------
// 地球重力定数の取得
double ib2_mss::Constants::ue()
{
	return instance().ue_;
}

//------------------------------------------------------------------------------
// 地球自転角速度の取得
double ib2_mss::Constants::we()
{
	return instance().we_;
}

//------------------------------------------------------------------------------
// 標準大気密度の取得
double ib2_mss::Constants::rho0()
{
	return instance().rho0_;
}

//------------------------------------------------------------------------------
// インスタンスの取得
ib2_mss::Constants& ib2_mss::Constants::instance()
{
	static Constants instance;
	return instance;
}

// End Of File -----------------------------------------------------------------
