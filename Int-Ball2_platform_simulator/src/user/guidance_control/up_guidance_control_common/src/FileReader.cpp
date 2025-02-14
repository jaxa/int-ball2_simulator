
#include "guidance_control_common/FileReader.h"
#include "guidance_control_common/Log.h"

#include <cstring>
#include <sstream>
#include <fstream>
#include <stdexcept>
#include <limits>

//------------------------------------------------------------------------------
// ファイルスコープ
namespace
{
	/** 不要文字 */
	const std::string OMISSION(" \t\r\n\ufeff");
	
	/** 文字列から値への変換エラーメッセージ
	 * @param [in] comment 例外のコメント
	 * @param [in] field 文字列フィールド
	 * @param [in] name 変数名
	 * @param [in] base 基数
	 */
	std::string errorConversion
	(const std::string& comment, const std::string& field,
	 const std::string& name, int base = -1)
	{
		std::string message
		(comment + " inputed(" + name + ")=\"" + field + "\"");
		if (base >= 0)
			message += ", base=" + std::to_string(base);
		return message;
	}
}

//------------------------------------------------------------------------------
// コンストラクタ
ib2_mss::FileReader::FileReader
(const std::string& filename, const std::string& comment, bool blank) :
filename_(filename), block_(0), line_(0)
{
	std::ios::sync_with_stdio(false);
	std::ifstream f(filename);
	if (f.fail())
	{
		std::string what("cannot open " + filename);
		LOG_ERROR(what);
		throw std::invalid_argument(what);
	}
	
	bool outblock(true);
	std::string line;
	while (std::getline(f, line))
	{
		std::string data(omit(line, "\r\n\ufeff"));
		std::string::size_type ic(data.find_first_of(comment));
		if (ic != std::string::npos)
			data = data.substr(0, ic);
		
		bool nodata(data.find_first_not_of(OMISSION) == std::string::npos);
		if (nodata)
			outblock = true;
		else if (outblock)
		{
			block_.push_back(line_.size());
			outblock = false;
		}
		if (blank || !nodata)
			line_.push_back(std::move(data));
	}
}

//------------------------------------------------------------------------------
// デストラクタ
ib2_mss::FileReader::~FileReader() = default;

//------------------------------------------------------------------------------
// ファイル名の取得
const std::string& ib2_mss::FileReader::filename() const
{
	return filename_;
}

//------------------------------------------------------------------------------
// ブロック開始行番号の参照
const std::vector<size_t>& ib2_mss::FileReader::block() const
{
	return block_;
}

//------------------------------------------------------------------------------
// 行数の取得
size_t ib2_mss::FileReader::lines() const
{
	return line_.size();
}

//------------------------------------------------------------------------------
// 指定ブロックの行数の取得
size_t ib2_mss::FileReader::lines(size_t blockno) const
{
	if (blockno >= block_.size())
		return 0;
	size_t blocknext(blockno + 1);
	size_t ie(blocknext < block_.size() ? block_.at(blocknext) : line_.size());
	return ie - block_.at(blockno);
}

//------------------------------------------------------------------------------
// 全行文字列の参照(trimしない)
const std::vector<std::string>& ib2_mss::FileReader::line() const
{
	return line_;
}

//------------------------------------------------------------------------------
// 全行文字列の取得
std::vector<std::string> ib2_mss::FileReader::string() const
{
	std::vector<std::string> v;
	v.reserve(line_.size());
	for (auto& line: line_)
		v.push_back(trim(line));
	return v;
}

//------------------------------------------------------------------------------
// 文字列の取得
std::string ib2_mss::FileReader::string
(size_t lineno, const std::string& comment) const
{
	std::string data(line_.at(lineno));
	if (!comment.empty())
	{
		std::string::size_type ic(data.find_first_of(comment));
		if (ic != std::string::npos)
			data = data.substr(0, ic);
	}
	return trim(data);
}

//------------------------------------------------------------------------------
// 倍精度浮動小数値の取得
double ib2_mss::FileReader::value
(size_t lineno, const std::string& name, bool empty0, bool strict) const
{
	return value(line_.at(lineno), name, empty0, strict);
}

//------------------------------------------------------------------------------
// 浮動小数値の取得
float ib2_mss::FileReader::valueFloat
(size_t lineno, const std::string& name, bool empty0, bool strict) const
{
	return valueFloat(line_.at(lineno), name, empty0, strict);
}

//------------------------------------------------------------------------------
// short型数値の取得
short ib2_mss::FileReader::valueShort
(size_t lineno, const std::string& name, int base,
 bool empty0, bool strict) const
{
	return valueShort(line_.at(lineno), name, base, empty0, strict);
}

//------------------------------------------------------------------------------
// int型数値の取得
int ib2_mss::FileReader::valueInt
(size_t lineno, const std::string& name, int base,
 bool empty0, bool strict) const
{
	return valueInt(line_.at(lineno), name, base, empty0, strict);
}

//------------------------------------------------------------------------------
// long型数値の取得
long ib2_mss::FileReader::valueLong
(size_t lineno, const std::string& name, int base,
 bool empty0, bool strict) const
{
	return valueLong(line_.at(lineno), name, base, empty0, strict);
}

//------------------------------------------------------------------------------
// longlong型数値の取得
long long ib2_mss::FileReader::valueLL
(size_t lineno, const std::string& name, int base,
 bool empty0, bool strict) const
{
	return valueLL(line_.at(lineno), name, base, empty0, strict);
}

//------------------------------------------------------------------------------
// unsigned short型数値の取得
unsigned short ib2_mss::FileReader::valueUS
(size_t lineno, const std::string& name, int base,
 bool empty0, bool strict) const
{
	return valueUS(line_.at(lineno), name, base, empty0, strict);
}

//------------------------------------------------------------------------------
// unsigned int型数値の取得
unsigned int ib2_mss::FileReader::valueUI
(size_t lineno, const std::string& name, int base,
 bool empty0, bool strict) const
{
	return valueUI(line_.at(lineno), name, base, empty0, strict);
}

//------------------------------------------------------------------------------
// unsigned long型数値の取得
unsigned long ib2_mss::FileReader::valueUL
(size_t lineno, const std::string& name, int base,
 bool empty0, bool strict) const
{
	return valueUL(line_.at(lineno), name, base, empty0, strict);
}

//------------------------------------------------------------------------------
// unsigned long long型数値の取得
unsigned long long ib2_mss::FileReader::valueULL
(size_t lineno, const std::string& name, int base,
 bool empty0, bool strict) const
{
	return valueULL(line_.at(lineno), name, base, empty0, strict);
}

//------------------------------------------------------------------------------
// 文字列の配列の取得
std::vector<std::string> ib2_mss::FileReader::vectorString
(size_t lineno, const std::string& delimiter, bool detectAll) const
{
	return vectorString(string(lineno), delimiter, detectAll);
}

//------------------------------------------------------------------------------
// 実数配列の取得
std::vector<double> ib2_mss::FileReader::vector
(size_t lineno, const std::string& delimiter, bool detectAll,
 bool empty0, bool strict) const
{
	auto fields(vectorString(lineno, delimiter, detectAll));
	return vector(fields, 0, 0, empty0, strict);
}

//------------------------------------------------------------------------------
// 不要文字列の除去
std::string ib2_mss::FileReader::trim(const std::string& field)
{
	std::string::size_type is(field.find_first_not_of(OMISSION));
	if (is == std::string::npos)
		return std::string();
	
	std::string::size_type ie(field.find_last_not_of(OMISSION));
	return field.substr(is, ie - is + 1);
}

//------------------------------------------------------------------------------
// 不要文字列の除去
std::string ib2_mss::FileReader::omit
(const std::string& field, const std::string& object)
{
	std::string::size_type is(field.find_first_not_of(object));
	std::string result;
	while (is != std::string::npos)
	{
		std::string::size_type ie(field.find_first_of(object, is));
		bool last(ie == std::string::npos);
		std::string::size_type n(last ? ie : ie - is);
		result += field.substr(is, n);
		is = last ? ie : ie + 1;
	}
	return result;
}

//------------------------------------------------------------------------------
// 特定文字列の抽出
std::string ib2_mss::FileReader::extract
(const std::string& field, const std::string& object)
{
	std::string::size_type is(field.find_first_of(object));
	std::string result;
	while (is != std::string::npos)
	{
		std::string::size_type ie(field.find_first_not_of(object, is));
		bool last(ie == std::string::npos);
		std::string::size_type n(last ? ie : ie - is);
		result += field.substr(is, n);
		is = last ? ie : ie + 1;
	}
	return result;
}

//------------------------------------------------------------------------------
// 文字列からdouble値への変換
double ib2_mss::FileReader::value
(const std::string& field, const std::string& name, bool empty0, bool strict)
{
	try
	{
		std::string str(trim(field));
		if (empty0 && str.empty())
			return 0.;
		size_t ie(str.length());
		auto x(std::stod(str, &ie));
		if (strict && ie < str.length())
			throw std::invalid_argument("invalid string:" + str.substr(ie));
		return x;
	}
	catch (const std::exception& e)
	{
		std::string message(Log::caughtException(e.what()));
		LOG_ERROR(errorConversion(message, field, name));
		throw;
	}
}

//------------------------------------------------------------------------------
// 文字列からfloat値への変換
float ib2_mss::FileReader::valueFloat
(const std::string& field, const std::string& name, bool empty0, bool strict)
{
	try
	{
		std::string str(trim(field));
		if (empty0 && str.empty())
			return 0.;
		size_t ie(str.length());
		auto x(std::stof(str, &ie));
		if (strict && ie < str.length())
			throw std::invalid_argument("invalid string:" + str.substr(ie));
		return x;
	}
	catch (const std::exception& e)
	{
		std::string message(Log::caughtException(e.what()));
		LOG_ERROR(errorConversion(message, field, name));
		throw;
	}
}

//------------------------------------------------------------------------------
// 文字列からshort値への変換
short ib2_mss::FileReader::valueShort
(const std::string& field, const std::string& name, int base,
 bool empty0, bool strict)
{
	try
	{
		std::string str(trim(field));
		if (empty0 && str.empty())
			return 0;
		size_t ie(str.length());
		auto x(std::stoi(field, &ie, base));
		if (strict && ie < str.length())
			throw std::invalid_argument("invalid string:" + str.substr(ie));
		if (x < std::numeric_limits<short>::min() ||
			x > std::numeric_limits<short>::max())
			throw std::out_of_range("overflowed short int");
		return static_cast<short>(x);
	}
	catch (const std::exception& e)
	{
		std::string message(Log::caughtException(e.what()));
		LOG_ERROR(errorConversion(message, field, name, base));
		throw;
	}
}

//------------------------------------------------------------------------------
// 文字列からint値への変換
int ib2_mss::FileReader::valueInt
(const std::string& field, const std::string& name, int base,
 bool empty0, bool strict)
{
	try
	{
		std::string str(trim(field));
		if (empty0 && str.empty())
			return 0;
		size_t ie(str.length());
		auto x(std::stoi(field, &ie, base));
		if (strict && ie < str.length())
			throw std::invalid_argument("invalid string:" + str.substr(ie));
		return x;
	}
	catch (const std::exception& e)
	{
		std::string message(Log::caughtException(e.what()));
		LOG_ERROR(errorConversion(message, field, name, base));
		throw;
	}
}

//------------------------------------------------------------------------------
// 文字列からlong値への変換
long ib2_mss::FileReader::valueLong
(const std::string& field, const std::string& name, int base,
 bool empty0, bool strict)
{
	try
	{
		std::string str(trim(field));
		if (empty0 && str.empty())
			return 0;
		size_t ie(str.length());
		auto x(std::stol(field, &ie, base));
		if (strict && ie < str.length())
			throw std::invalid_argument("invalid string:" + str.substr(ie));
		return x;
	}
	catch (const std::exception& e)
	{
		std::string message(Log::caughtException(e.what()));
		LOG_ERROR(errorConversion(message, field, name, base));
		throw;
	}
}

//------------------------------------------------------------------------------
// 文字列からlong long値への変換
long long ib2_mss::FileReader::valueLL
(const std::string& field, const std::string& name, int base,
 bool empty0, bool strict)
{
	try
	{
		std::string str(trim(field));
		if (empty0 && str.empty())
			return 0;
		size_t ie(str.length());
		auto x(std::stoll(field, &ie, base));
		if (strict && ie < str.length())
			throw std::invalid_argument("invalid string:" + str.substr(ie));
		return x;
	}
	catch (const std::exception& e)
	{
		std::string message(Log::caughtException(e.what()));
		LOG_ERROR(errorConversion(message, field, name, base));
		throw;
	}
}

//------------------------------------------------------------------------------
// 文字列からunsigned short値への変換
unsigned short ib2_mss::FileReader::valueUS
(const std::string& field, const std::string& name, int base,
 bool empty0, bool strict)
{
	try
	{
		std::string str(trim(field));
		if (empty0 && str.empty())
			return 0;
		size_t ie(str.length());
		auto x(std::stoul(field, &ie, base));
		if (strict && ie < str.length())
			throw std::invalid_argument("invalid string:" + str.substr(ie));
		if (x > std::numeric_limits<unsigned short>::max())
			throw std::out_of_range("overflowed unsigned short");
		return static_cast<unsigned short>(x);
	}
	catch (const std::exception& e)
	{
		std::string message(Log::caughtException(e.what()));
		LOG_ERROR(errorConversion(message, field, name, base));
		throw;
	}
}

//------------------------------------------------------------------------------
// 文字列からunsigned int値への変換
unsigned int ib2_mss::FileReader::valueUI
(const std::string& field, const std::string& name, int base,
 bool empty0, bool strict)
{
	try
	{
		std::string str(trim(field));
		if (empty0 && str.empty())
			return 0;
		size_t ie(str.length());
		auto x(std::stoul(field, &ie, base));
		if (strict && ie < str.length())
			throw std::invalid_argument("invalid string:" + str.substr(ie));
		if (x > std::numeric_limits<unsigned int>::max())
			throw std::out_of_range("overflowed unsigned int");
		return static_cast<unsigned int>(x);
	}
	catch (const std::exception& e)
	{
		std::string message(Log::caughtException(e.what()));
		LOG_ERROR(errorConversion(message, field, name, base));
		throw;
	}
}

//------------------------------------------------------------------------------
// 文字列からunsigned long値への変換
unsigned long ib2_mss::FileReader::valueUL
(const std::string& field, const std::string& name, int base,
 bool empty0, bool strict)
{
	try
	{
		std::string str(trim(field));
		if (empty0 && str.empty())
			return 0;
		size_t ie(str.length());
		auto x(std::stoul(field, &ie, base));
		if (strict && ie < str.length())
			throw std::invalid_argument("invalid string:" + str.substr(ie));
		return x;
	}
	catch (const std::exception& e)
	{
		std::string message(Log::caughtException(e.what()));
		LOG_ERROR(errorConversion(message, field, name, base));
		throw;
	}
}

//------------------------------------------------------------------------------
// 文字列からunsigned long long値への変換
unsigned long long ib2_mss::FileReader::valueULL
(const std::string& field, const std::string& name, int base,
 bool empty0, bool strict)
{
	try
	{
		std::string str(trim(field));
		if (empty0 && str.empty())
			return 0;
		size_t ie(str.length());
		auto x(std::stoull(field, &ie, base));
		if (strict && ie < str.length())
			throw std::invalid_argument("invalid string:" + str.substr(ie));
		return x;
	}
	catch (const std::exception& e)
	{
		std::string message(Log::caughtException(e.what()));
		LOG_ERROR(errorConversion(message, field, name, base));
		throw;
	}
}

//------------------------------------------------------------------------------
// 文字列から文字列vector配列の取得
std::vector<std::string> ib2_mss::FileReader::vectorString
(const std::string& field, const std::string& delimiter, bool detectAll)
{
	std::string::size_type is(detectAll? 0: field.find_first_not_of(delimiter));
	std::vector<std::string> array;
	while (is != std::string::npos)
	{
		std::string::size_type ie(field.find_first_of(delimiter, is));
		std::string::size_type n(ie == std::string::npos ? ie : ie - is);
		array.push_back(trim(field.substr(is, n)));
		if (ie == std::string::npos)
			break;
		if (detectAll)
			is = ie + 1;
		else
			is = field.find_first_not_of(delimiter, ie);
	}
	return array;
}

//------------------------------------------------------------------------------
// 文字列ベクタからdouble値ベクタへの変換
std::vector<double> ib2_mss::FileReader::vector
(const std::vector<std::string>& fields, size_t is, size_t ie,
 bool empty0, bool strict)
{
	size_t n(fields.size());
	if (ie <= is || ie >= n)
		ie = n - 1;
	std::vector<double> v;
	v.reserve(n);
	for (size_t i = is; i <= ie; ++i)
	{
		try
		{
			v.push_back(value(fields.at(i), "", empty0, strict));
		}
		catch (const std::exception& e)
		{
			LOG_ERROR(Log::errorArrayIndex(e.what(), "", "fields", i, n));
			throw;
		}
	}
	return v;
}

//------------------------------------------------------------------------------
// 文字列ベクタからint値ベクタへの変換
std::vector<int> ib2_mss::FileReader::vectorInt
(const std::vector<std::string>& fields, size_t is, size_t ie, int base,
 bool empty0, bool strict)
{
	size_t n(fields.size());
	if (ie <= is || ie >= n)
		ie = n - 1;
	std::vector<int> v;
	v.reserve(n);
	for (size_t i = is; i <= ie; ++i)
	{
		try
		{
			v.push_back(valueInt(fields.at(i), "", base, empty0, strict));
		}
		catch (const std::exception& e)
		{
			LOG_ERROR(Log::errorArrayIndex(e.what(), "", "fields", i, n));
			throw;
		}
	}
	return v;
}

//------------------------------------------------------------------------------
// 文字列ベクタからunsigned long値ベクタへの変換
std::vector<unsigned long> ib2_mss::FileReader::vectorUL
(const std::vector<std::string>& fields, size_t is, size_t ie, int base,
 bool empty0, bool strict)
{
	size_t n(fields.size());
	if (ie <= is || ie >= fields.size())
		ie = n - 1;
	std::vector<unsigned long> v;
	v.reserve(n);
	for (size_t i = is; i <= ie; ++i)
	{
		try
		{
			v.push_back(valueUL(fields.at(i), "", base, empty0, strict));
		}
		catch (const std::exception& e)
		{
			LOG_ERROR(Log::errorArrayIndex(e.what(), "", "fields", i, n));
			throw;
		}
	}
	return v;
}

// End Of File -----------------------------------------------------------------
