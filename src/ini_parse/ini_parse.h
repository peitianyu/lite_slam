#ifndef __INI_PARSE_H__
#define __INI_PARSE_H__

#include<iostream>
#include<fstream>
#include<sstream>


// FIXME: 不够优雅, 应该在构造的时候储存下map<section, map<key, value>>,后使用单例来调
class IniParse
{
public:
	struct Param
	{
		char comment = '#';
		char separator = '=';
	};

	IniParse(const Param& p)
	:m_param(p)
	{}

 	template<typename DataType>
	bool GetValue(const std::string& filename, const std::string& section, const std::string& key, DataType& value)
	{
		std::ifstream ifs(filename);
		if(!ifs.is_open()) return false;

		std::string section_tmp = '[' + section + ']';

		std::string line;
		while(std::getline(ifs, line))
		{
			std::string::size_type comment_id = line.find_first_of(m_param.comment);
			if(comment_id != std::string::npos && comment_id < line.find(section_tmp)) continue;

			if(line.find(section_tmp) != std::string::npos)
			{
				while(std::getline(ifs, line))
				{
					if(line.find(section_tmp) != std::string::npos){
						ifs.close();
						return false;
					}

					std::string::size_type key_id = line.find(key);
					if(key_id == std::string::npos) continue;

					comment_id = line.find_first_of(m_param.comment);
					if(comment_id != std::string::npos && comment_id < key_id) continue;

					std::string::size_type separator_id = line.find(m_param.separator);
					if(separator_id == std::string::npos) continue;
					
					line.erase(0, separator_id + 1);
					std::istringstream iss(line);
					iss >> value;

					ifs.close();
					return true;
				}
			}
		}
		ifs.close();
		return false;
	}

private:
	Param m_param;
};


#endif // __INI_PARSE_H__