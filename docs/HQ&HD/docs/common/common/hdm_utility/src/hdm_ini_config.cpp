#include <stdlib.h>
#include <string.h>
#include <memory>
#include "hdm_utility/hdm_ini_config.h"


namespace hdm_utility
{	

CHdmIniConfig* CHdmIniConfig::GetInstance()
{
    static CHdmIniConfig hdm_config;
    return &hdm_config;
}

CHdmIniConfig::CHdmIniConfig()
: file_path_()
, ini_config_values_()
{
    std::map<std::string, std::string>   config_value;
    config_value["MinDataRange"] = "-10000";
    config_value["MaxDataRange"] = "20000";
    ini_config_values_["ToEmData"] = config_value;    

    config_value.clear();
    config_value["LogLevel"] = "1";
    config_value["ConsoleOutput"] = "1";
    config_value["FileOutput"] = "1";    
    ini_config_values_["DbgLog"] = config_value;    
 
}

CHdmIniConfig::~CHdmIniConfig()
{
    
}

bool CHdmIniConfig::Initialize(const std::string &file_path)
{
    file_path_ = file_path;
    return ParseIniFile(file_path_);
}

std::string CHdmIniConfig::GetIniConfigPath()
{
    return file_path_;
}

const std::string &CHdmIniConfig::GetConfigValueString(const char *section_name, const char *config_name, const std::string &default_value)
{
    const auto it_ini_section = ini_config_values_.find(section_name);
    if (it_ini_section == ini_config_values_.end()) {
        printf("CHdmIniConfig::GetConfigValueString find by section name failed, section_name[%s]\n", section_name);
        return default_value;
    }

    const auto it_config = it_ini_section->second.find(config_name);
    if (it_config == it_ini_section->second.end()) {
        printf("CHdmIniConfig::GetConfigValueString find by config name failed, section_name[%s], config_name[%s]\n", section_name, config_name);
        return default_value;
    }
    return it_config->second;
}

int32_t CHdmIniConfig::GetConfigValueInt(const char *section_name, const char *config_name, const int32_t default_value)
{
    const std::string str_default_value = "not_find";
    const std::string &str_value = GetConfigValueString(section_name, config_name, str_default_value);
    if (str_default_value == str_value) {
        return default_value;
    }
    return atoi(str_value.c_str());
}

int64_t CHdmIniConfig::GetConfigValueInt64(const char *section_name, const char *config_name, const int64_t default_value)
{
    const std::string str_default_value = "not_find";
    const std::string &str_value = GetConfigValueString(section_name, config_name, str_default_value);
    if (str_default_value == str_value) {
        return default_value;
    }

    char *end = nullptr;
    return strtoll(str_value.c_str(), &end, 10);
}

float32_t CHdmIniConfig::GetConfigValueFloat(const char *section_name, const char *config_name, const float32_t default_value)
{
    const std::string str_default_value = "not_find";
    const std::string &str_value = GetConfigValueString(section_name, config_name, str_default_value);
    if (str_default_value == str_value) {
        return default_value;
    }
    return atof(str_value.c_str());
}

bool CHdmIniConfig::ParseIniFile(const std::string &ini_file_path)
{  
	FILE *file = fopen(ini_file_path.c_str(), "r");
    if (nullptr == file)   {	
        printf("CHdmIniConfig::ParseIniFile open file failed, ini_file_path[%s]\n", ini_file_path.c_str());			
        return false;
    }

    fseek(file, 0, SEEK_END);
    long ftell_result = ftell(file);
    if (0 > ftell_result) {
        fclose(file);
        file = nullptr;
        return false;
    }
    size_t file_size = ftell_result;
    if (file_size <= 7) {
        printf("CHdmIniConfig::ParseIniFile file size[%lu], ini_file_path[%s]\n", file_size, ini_file_path.c_str());	
        fclose(file);
        file = nullptr;		
        return false; 
    }
    fseek(file, 0, SEEK_SET);
    
    std::shared_ptr<char> sp_buffer = std::shared_ptr<char>(new char[file_size + 1]);    
    if (nullptr == sp_buffer) {
        printf("CHdmIniConfig::ParseIniFile allocate failed, file_size[%lu]\n", file_size);
        fclose(file);
        file = nullptr;		
        return false;
    }
    char *buffer = sp_buffer.get();

    int32_t	read_size = fread(buffer, 1, file_size, file);
	if (read_size != (int32_t)file_size) {	
        printf("CHdmIniConfig::ParseIniFile read file failed, read_size[%d], file_size[%lu]\n", read_size, file_size); 
        fclose(file);
        file = nullptr;      
		return false;
	}
    buffer[file_size] = '\0';
    fclose(file);
    file = nullptr;
    
	int32_t start_index = 0;
	// check encoded file or not.
	if (file_size >= 4 
        && buffer[0] == '\x6F' 
        && buffer[1] == '\x7F'
        && buffer[2] == '\x8F' 
        && buffer[3] == '\x9F')	{
		if (!Decode(buffer + 4, file_size - 4))		{       
			return false;
		}
		start_index += 4; //skip header
	}

	// check BOM and skip if found.
	if ((file_size - start_index >= 3) 
       && buffer[start_index + 0] == '\xEF' 
       && buffer[start_index + 1] == '\xBB' 
       && buffer[start_index + 2] == '\xBF')	{
		start_index += 3;
	}
    if ((int32_t)file_size > start_index) {
        return ParseIniBuffer(buffer + start_index, file_size - start_index);
    }
    else {
        printf("CHdmIniConfig::ParseIniFile start_index failed, start_index[%d], file_size[%lu]\n", start_index, file_size);        	
        return false;
    }	
}


bool CHdmIniConfig::ParseIniBuffer(const char *buffer, size_t size)
{
	std::string	cur_section = "Default";

	for (char *line_first = strtok((char *)buffer, "\r\n"); nullptr != line_first; line_first = strtok(nullptr, "\r\n")) {
		char	*line_end = line_first + strlen(line_first);
		RemoveSpace(line_first, line_end);
		if (line_end - line_first <= 1) {
			continue;
		}
		else if (*line_first == '#') {
			continue;
		}
		else if (*line_first == '[') {
			if (*(line_end - 1) != ']') {
				//ERR << "Failed to parse line: " << string(line_first, line_end);
				//REPORT_CONFIG_READING_DTC(dtc_reported, DCT_CONFIG_FILE_READ_ERROR);
				return false;
			}
			line_first++;
			line_end--;
			RemoveSpace(line_first, line_end);
			cur_section.assign(line_first, line_end);
		}
		else {
			char *eq = line_first;
			for (; eq < line_end && *eq != '='; ++eq) {}
			if (*eq != '=') {
				//ERR << "Failed to parse line: " << string(line_first, line_end);
				//REPORT_CONFIG_READING_DTC(dtc_reported, DCT_CONFIG_FILE_READ_ERROR);
				return false;
			}

			char	*key_first = line_first;
			char	*key_end = eq;
			RemoveSpace(key_first, key_end);
			std::string	key;
			key.assign(key_first, key_end);

			char	*value_first = eq + 1;
			char	*value_end = line_end;
			RemoveSpace(value_first, value_end);
			std::string	value;
			value.assign(value_first, value_end);
			ini_config_values_[cur_section][key] = value;
		}
	}
	return true;
}

bool CHdmIniConfig::Decode(char *data, size_t size)
{
    if(nullptr == data || size <= 0)    {
        return false;
    }
    
    char first = '\xEF';
    for(char *current = data, key = first; size > 0; current++, size--)    {
        char origin = *current;
        *current = *current ^ key;
        key ^= origin;
    }
    return true;
}

void CHdmIniConfig::RemoveSpace(char *&first, char *&end)
{
	for (; *first == ' ' && *first != '\0'; ++first) {}
	for (; end > first && *(end - 1) == ' '; --end) {}
}

void CHdmIniConfig::Output()
{
    printf("CHdmIniConfig::Output  file_path_[%s]\n", file_path_.c_str());
	for (auto it_section = ini_config_values_.begin(); it_section != ini_config_values_.end(); ++it_section) {
        printf("[%s]\n", it_section->first.c_str());
        for (auto it_config = it_section->second.begin(); it_config != it_section->second.end(); ++it_config) {
            printf("%s=%s\n", it_config->first.c_str(), it_config->second.c_str());
        }
    }
}

}
/* EOF */