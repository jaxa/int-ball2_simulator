
#include "guidance_control_common/Log.h"
#include "guidance_control_common/FileWriter.h"

#include <ros/ros.h>

#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <ctime>
#include <chrono>

#ifndef __WIN32__
#include <unistd.h>
#endif

//------------------------------------------------------------------------------
// 定数
namespace
{
	/** ログレベルの文字列 */
	const std::vector<std::string> LEVEL_STRING
	{
		"DEBUG", "INFO", "WARN", "ERROR"
	};
	
	/** ログレベルの探索.
	 * @param [in] input ログレベル文字列
	 * @return ログレベル
	 * @throw invalid_argument ログレベル文字列が不正
	 */
	ib2_mss::Log::LEVEL searchLevel(const std::string& input)
	{
		size_t level(0);
		for (auto& defined: LEVEL_STRING)
		{
			if (!input.empty() && defined.find(input) == 0)
				return static_cast<ib2_mss::Log::LEVEL>(level);
			++level;
		}
		throw std::invalid_argument("invalid log level string:" + input);
	}
	
	/** エラー出力.
	 * @param [in] message	ログメッセージ
	 * @param [in] level	ログレベル
	 * @param [in] file		ファイル名
	 * @param [in] lineno	行番号
	 */
	void outcerr(const std::string& message, const std::string& level,
				 const std::string& file, unsigned long lineno)
	{
		std::cerr << "message=" << message << ", level=" << level;
		std::cerr << ", file=" << file << ", line=" << lineno << std::endl;
	}
	
	/** 現在時刻文字列.
	 * @param file ファイル名用文字列生成フラグ
	 * @return 現在時刻文字列[YYYY/MM/DD hh:mm:ss.fff]ファイル用[YYYYMMDD_hhmmss]
	 */
	std::string currentTime(bool file = false)
	{
		using namespace std::chrono;
		auto now(system_clock::now());
		time_t tt(system_clock::to_time_t(now));
		std::tm lt;
#ifdef __WIN32__
		localtime_s(&lt, &tt);
#else	//__WIN32__
		localtime_r(&tt, &lt);
#endif	//__WIN32__
		std::string time_form(file ? "%Y%m%d_%H%M%S" : "%Y/%m/%d %H:%M:%S");
		std::ostringstream ssnow;
#if defined(__clang__) || (defined(__GNUC__) && (__GNUC__ >= 5))
		ssnow << std::put_time(&lt, time_form.c_str());
#else	//__clang__
		static const int TIME_STRING_LENGTH(40);
		char tmstring[TIME_STRING_LENGTH];
		strftime(tmstring, TIME_STRING_LENGTH, time_form.c_str(), &lt);
		ssnow << tmstring;
#endif	//__clang__
		if (file)
			return ssnow.str();
		
		auto sec(time_point_cast<seconds>(now));
		auto usec(duration_cast<microseconds>(now - sec).count());
		ssnow << "." << std::setw(6) << std::setfill('0') << usec;
		return ssnow.str();
	}
}

//------------------------------------------------------------------------------
// 実装クラス
/**
 * @brief ログ実装クラス
 */
class ib2_mss::Log::Implement
{
	//--------------------------------------------------------------------------
	// コンストラクタ/デストラクタ
private:
	/** デフォルトコンストラクタ. */
	Implement() :
	maxlines_(DEFAULT_MAX_LINES), lines_(0),
	separated_(), original_(), logfile_(), level_(LEVEL::ERROR_LOG)
	{
	}
	
	/** デストラクタ */
	~Implement() = default;
	
	//--------------------------------------------------------------------------
	// コピー/ムーブ
private:
	/** コピーコンストラクタ. */
	Implement(const Implement&) = delete;
	
	/** コピー代入演算子. */
	Implement& operator=(const Implement&) = delete;
	
	/** ムーブコンストラクタ. */
	Implement(Implement&&) = delete;
	
	/** ムーブ代入演算子. */
	Implement& operator=(Implement&&) = delete;
	
	//--------------------------------------------------------------------------
	// 操作(Setter)
public:
	/** ログの構成.
	 * ログ初期化ファイルを読み込み、ログ出力条件を構築する。
	 * @param [in] logfile ログファイル名
	 * @param [in] loglevel ログレベル
	 * @param [in] maxlines 最大書き込み行数
	 * @retval true ログ初期化成功
	 * @retval false ログ初期化失敗
	 */
	bool configure
	(const std::string& logfile, const std::string& loglevel, size_t maxlines)
	{
		try
		{
			separated_.clear();
			original_ .clear();
			logfile_  .clear();
			level_     = searchLevel(loglevel);
			maxlines_  = maxlines;
			original_  = logfile;
			separated_ = FileWriter::separate(logfile, true);
			updateLogFile();
			return true;
		}
		catch (const std::exception& e)
		{
			outcerr("failure to configure", loglevel, logfile, maxlines);
			return false;
		}
	}
	
private:
	/** ログ出力ファイルの更新 */
	void updateLogFile()
	{
		logfile_.clear();
		lines_ = 0;
		const std::string& body(separated_.at(1));
		if (body.empty())
			throw std::invalid_argument("empty log file name");
		std::string now(currentTime(true));
		std::string linkfile(body + "_" + now + separated_.back());
		std::string logfile(separated_.front() + linkfile);
		FileWriter f(logfile, true);
#ifdef __WIN32__
//		std::string command("mklink " + original_ + " " + logfile);
//		std::system(command.c_str());
#else
		unlink(original_.c_str());
		int code = symlink(linkfile.c_str(), original_.c_str());
		if (code != 0)
			outcerr("failure to link", LEVEL_STRING[2], __FILE__, __LINE__);
#endif	//__WIN32__
		logfile_ = std::move(logfile);
	}
	
	//--------------------------------------------------------------------------
	// 属性(Getter)
public:
	/** インスタンスの取得(シングルトンパターン).
	 * @return ログクラスのインスタンスへの参照
	 */
	static Implement& instance()
	{
		static Implement instance;
		return instance;
	}
	
	/** ログファイル名の参照
	 * @return ログファイル名
	 */
	const std::string& logfile() const
	{
		return logfile_;
	}
	
	/** ログ出力判定.
	 * @param [in] level ログレベル
	 * @retval true  ログを出力する
	 * @retval false ログを出力しない
	 */
	bool enabled(LEVEL level) const
	{
		return !logfile_.empty() && level >= level_;
	}
	
	//--------------------------------------------------------------------------
	// 実装
public:
	/** ログ出力.
	 * @param [in] level	ログレベル
	 * @param [in] message	ログメッセージ
	 * @param [in] file		ファイル名
	 * @param [in] function	関数名
	 * @param [in] lineno	行番号
	 */
	void write(LEVEL level, const std::string& message, const std::string& file,
			   const std::string& function, unsigned long lineno)
	{
//		if (level >= level_)
//		{
//			const std::string& strlv(LEVEL_STRING[static_cast<size_t>(level)]);
//			try
//			{
//				if (logfile_.empty())
//					throw std::domain_error("logfile is empty");
				size_t is(file.find_last_of("/\\"));
				std::ostringstream ss;
//				ss << currentTime();
//				ss << " [" << std::setw(5) << std::left << strlv << "] ";
				ss << file.substr(is + 1) << " #";
				ss << std::setw(5) << std::right << lineno << " | ";
				ss << function << " : " << message;
				
				if (level == LEVEL::DEBUG_LOG)
					ROS_DEBUG("%s", ss.str().c_str());
				else if(level == LEVEL::INFO_LOG)
					ROS_INFO("%s", ss.str().c_str());
				else if(level == LEVEL::WARN_LOG)
					ROS_WARN("%s", ss.str().c_str());
				else
					ROS_ERROR("%s", ss.str().c_str());
				
//				if (lines_ >= maxlines_)
//					updateLogFile();
//				FileWriter f(logfile_, true);
//				f.write(ss.str());
//				++lines_;
//			}
//			catch (const std::exception& e)
//			{
//				outcerr(message, strlv, file, lineno);
//			}
//		}
	}
	
	//--------------------------------------------------------------------------
	// メンバー変数
private:
	/** 最大書き込み行数 */
	size_t maxlines_;
	
	/** 書き込み行数 */
	size_t lines_;
	
	/** 分割ログファイル名 */
	std::vector<std::string> separated_;
	
	/** 元のログファイル */
	std::string original_;
	
	/** ログファイル */
	std::string logfile_;
	
	/** ログレベル */
	LEVEL level_;
};

//------------------------------------------------------------------------------
// 定数定義
const std::string ib2_mss::Log::CAUGHT_EXCEPTION("caught exception:");

//------------------------------------------------------------------------------
// ログの構成
bool ib2_mss::Log::configure(const std::string& logfile,
						  const std::string& loglevel, size_t maxlines)
{
	return Implement::instance().configure(logfile, loglevel, maxlines);
}

//------------------------------------------------------------------------------
// ログファイル名の参照
const std::string& ib2_mss::Log::logfile()
{
	return Implement::instance().logfile();
}

//------------------------------------------------------------------------------
// デバッグ出力判定
bool ib2_mss::Log::isDebug()
{
	return Implement::instance().enabled(LEVEL::DEBUG_LOG);
}

//------------------------------------------------------------------------------
// デバッグログ出力
void ib2_mss::Log::debug
(const std::string& message, const std::string& file,
 const std::string& function, unsigned long lineno)
{
	Implement::instance().write(LEVEL::DEBUG_LOG,message,file,function,lineno);
}

//------------------------------------------------------------------------------
// 情報ログ出力
void ib2_mss::Log::info
(const std::string& message, const std::string& file,
 const std::string& function, unsigned long lineno)
{
	Implement::instance().write
	(LEVEL::INFO_LOG, message, file, function, lineno);
}

//------------------------------------------------------------------------------
// 警告ログ出力
void ib2_mss::Log::warn
(const std::string& message, const std::string& file,
 const std::string& function, unsigned long lineno)
{
	Implement::instance().write(LEVEL::WARN_LOG,message,file,function,lineno);
}

//------------------------------------------------------------------------------
// エラーログ出力
void ib2_mss::Log::error
(const std::string& message, const std::string& file,
 const std::string& function, unsigned long lineno)
{
	Implement::instance().write(LEVEL::ERROR_LOG,message,file,function,lineno);
}

//------------------------------------------------------------------------------
// ファイル読み込みエラーメッセージ
std::string ib2_mss::Log::caughtException
(const std::string& what, const std::string& comment)
{
	std::string message(Log::CAUGHT_EXCEPTION + what);
	if (!comment.empty())
		message += "," + comment;
	return message;
}

//------------------------------------------------------------------------------
// ファイル読み込みエラーメッセージ
std::string ib2_mss::Log::errorFileLine
(const std::string& what, const std::string& comment,
 const std::string& filename, size_t lineno)
{
	std::string message(caughtException(what, comment));
	message += ", filename:" + filename + ", lineno:" + std::to_string(lineno);
	return message;
}

//------------------------------------------------------------------------------
// 配列読み込みエラーメッセージ
std::string ib2_mss::Log::errorArrayIndex
(const std::string& what, const std::string& comment,
 const std::string& name, size_t index, size_t size)
{
	std::string message(caughtException(what, comment) + ", name:" + name);
	message += ", index:" + std::to_string(index);
	message += ", size:"  + std::to_string(size);
	return message;
}

// End Of File -----------------------------------------------------------------
