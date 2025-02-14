
#include "guidance_control_common/FileWriter.h"
#include "guidance_control_common/Log.h"

#include <sstream>
#include <fstream>
#include <stdexcept>
#include <sys/stat.h>
#include <dirent.h>
#include <algorithm>

#ifdef __WIN32__
#include <direct.h>
#endif

//------------------------------------------------------------------------------
// ファイルスコープ
namespace
{
	/** ディレクトリ区切り文字 */
	const char *DIR_DELIMITER("/\\");
	
	/** ディレクトリ/ファイル名として不正な文字 */
	const char *INVALID_CHAR(":;|,*?<>\"");
}

//------------------------------------------------------------------------------
// コンストラクタ
ib2_mss::FileWriter::FileWriter(const std::string& filename, bool append) :
filename_(filename)
{
	if (invalidname(filename_, true))
		throw std::invalid_argument("invalid file name");
	std::vector<std::string> separated(separate(filename_));
	makedir(separated.front());
	std::ofstream f(filename_, append ? std::ios::app : std::ios::trunc);
	if (f.fail())
	{
		std::string what("failure to open file");
		LOG_ERROR(what + ":" + filename_);
		throw std::invalid_argument(what);
	}
}

//------------------------------------------------------------------------------
// デストラクタ
ib2_mss::FileWriter::~FileWriter() = default;

//------------------------------------------------------------------------------
// ファイルの出力
void ib2_mss::FileWriter::write(const std::string& line) const
{
	std::ofstream f(filename_, std::ios::app);
	f << line << std::endl;
}

//------------------------------------------------------------------------------
// ディレクトリを作成する
void ib2_mss::FileWriter::makedir(std::string& dirname)
{
	if (invalidname(dirname, true))
		throw std::invalid_argument("invalid directory name");
	if (dirname.find_last_of(DIR_DELIMITER) != dirname.size() - 1)
		dirname += "/";
	
	std::vector<std::string> dirlayer;
	std::string::size_type ie(0);
	bool first(true);
	while (ie != std::string::npos)
	{
		std::string::size_type is(first ? ie : ie + 1);
		ie = dirname.find_first_of(DIR_DELIMITER, is);
		if (ie != std::string::npos)
			dirlayer.push_back(dirname.substr(is, ie - is + 1));
		first = false;
	}
	
	std::string dir;
	for (auto& layer: dirlayer)
	{
		dir += layer;
		struct stat stat_buf;
		if (!dir.empty() && stat(dir.c_str(), &stat_buf) != 0)
		{
#ifdef __WIN32__
			int res = mkdir(dir.c_str());
#else
			int res = mkdir(dir.c_str(), 0777);
#endif
			if (res != 0)
			{
				std::string what("failure to create directory");
				LOG_ERROR(what + ":" + dir);
				throw std::invalid_argument(what);
			}
		}
	}
}

//------------------------------------------------------------------------------
// ディレクトリの中のファイルを削除する
void ib2_mss::FileWriter::deleteFiles(const std::string& dirname)
{
	std::string d(dirname.find_last_of(DIR_DELIMITER) != dirname.size() - 1 ?
				  "/" : "");
	auto files(listFiles(dirname));
	for (auto& file: files)
		remove((dirname + d + file).c_str());
}

//------------------------------------------------------------------------------
// 指定ディレクトリに存在するファイル名リストの取得
std::vector<std::string> ib2_mss::FileWriter::listFiles(const std::string& dirname)
{
	DIR* dp = opendir(dirname.c_str());
	if (dp == nullptr)
	{
		LOG_INFO("There is no directory \"" + dirname + "\"");
		return std::vector<std::string>(0);
	}
	std::vector<std::string> files;
	struct dirent* dent;
	while ((dent = readdir(dp)) != nullptr)
	{
		std::string file(dent->d_name);
		if (file != "." && file != "..")
			files.push_back(file);
	}
	closedir(dp);
	std::sort(files.begin(), files.end());
	return files;
}

//------------------------------------------------------------------------------
// ファイル名をディレクトリ名とファイル名と拡張子に分割する
std::vector<std::string> ib2_mss::FileWriter::separate
(const std::string& filename, bool extention)
{
	std::vector<std::string> separated;
	separated.reserve(3);
	
	auto slash(filename.find_last_of(DIR_DELIMITER));
	if (slash != std::string::npos)
		separated.push_back(filename.substr(0, ++slash));
	else
	{
		separated.push_back("./");
		slash = 0;
	}
	
	std::string bodyext(filename.substr(slash));
	if (!extention)
		separated.push_back(bodyext);
	else
	{
		auto dot(bodyext.find_last_of("."));
		if (dot == std::string::npos)
		{
			separated.push_back(bodyext);
			separated.push_back("");
		}
		else
		{
			separated.push_back(bodyext.substr(0, dot));
			separated.push_back(bodyext.substr(dot));
		}
	}
	return separated;
}

//------------------------------------------------------------------------------
// ファイル名の検査
bool ib2_mss::FileWriter::invalidname(const std::string& filename, bool fullpath)
{
	static const std::string MESSAGE("invalid file name :\"");
	if (!fullpath &&
		filename.find_first_of(DIR_DELIMITER) != std::string::npos)
	{
		LOG_WARN(MESSAGE + filename +
				 "\" includes directory delimiter char : " + DIR_DELIMITER);
		return true;
	}
	if (filename.find_first_of(INVALID_CHAR) != std::string::npos)
	{
		LOG_WARN(MESSAGE + filename +
				 "\" includes invalid char : " + INVALID_CHAR);
		return true;
	}
	return false;
}
// End Of File -----------------------------------------------------------------
