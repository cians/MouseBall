
#ifndef VISUALZATION_UTILITY_H
#define VISUALZATION_UTILITY_H

namespace Visualization {

    namespace IO
    {

        inline std::string ExtractFileDirectory(const std::string &fileName)
        {
            const std::string::size_type i1 = fileName.rfind('/'), i2 = fileName.rfind('\\');
            if(i1 == std::string::npos && i2 == std::string::npos)
                return std::string();
            else if(i1 != std::string::npos && i2 == std::string::npos)
                return fileName.substr(0, i1 + 1);
            else if(i1 == std::string::npos && i2 != std::string::npos)
                return fileName.substr(0, i2 + 1);
            else if(i1 > i2)
                return fileName.substr(0, i1 + 1);
            else
                return fileName.substr(0, i2+1);
        }

        inline std::string RemoveFileDirectory(const std::string &fileName)
        {
            const std::string::size_type i1 = fileName.rfind('/'), i2 = fileName.rfind('\\');
            if(i1 == std::string::npos && i2 == std::string::npos)
                return fileName;
            else if(i1 != std::string::npos && i2 == std::string::npos)
                return fileName.substr(i1 + 1, fileName.size());
            else if(i1 == std::string::npos && i2 != std::string::npos)
                return fileName.substr(i2 + 1, fileName.size());
            else if(i1 > i2)
                return fileName.substr(i1 + 1, fileName.size());
            else
                return fileName.substr(i2+1, fileName.size());
        }

        inline std::string ExtractFileExtension(const std::string &fileName)
        {
            const std::string::size_type i = fileName.rfind('.');
            if(i == std::string::npos)
                return std::string();
            else
                return fileName.substr(i + 1, fileName.size());
        }

        inline std::string RemoveFileExtension(const std::string &fileName)
        {
            const std::string::size_type i = fileName.rfind('.');
            if(i == std::string::npos)
                return fileName;
            else
                return fileName.substr(0, i);
        }

        inline std::string RemoveFileDirectoryAndExtension(const std::string &fileName)
        {
            RemoveFileDirectory(RemoveFileExtension(fileName));
        }

        inline std::string ReplaceFileDirectory(const std::string &fileName, const std::string &dirSrc, const std::string &dirDst)
        {
            if(fileName.find(dirSrc) != 0)
                return fileName;
            else
                return dirDst + fileName.substr(dirSrc.length(), fileName.length());
        }

        inline std::string InsertSuffix(const std::string &fileName, const std::string &suffix)
        {
            const std::string::size_type i = fileName.rfind('.');
            if(i == std::string::npos)
                return fileName + suffix;
            else
                return fileName.substr(0, i) + suffix + fileName.substr(i, fileName.length());
        }

        inline int ExtractFileNumber(const std::string &fileName)
        {
            int i2 = int(fileName.length());
            while(--i2 >= 0 && !isdigit(fileName[i2]));
            int i1 = ++i2;
            while(--i1 >= 0 && isdigit(fileName[i1]));
            if(++i1 == i2)
                return -1;
            else
                return atoi(fileName.substr(i1, i2 - i1).c_str());
        }

        inline std::string IncreaseFileNumber(const std::string &fileName, const int &incr)
        {
            const int len = int(fileName.length());
            int i2 = len;
            while(--i2 >= 0 && !isdigit(fileName[i2]));
            int i1 = ++i2;
            while(--i1 >= 0 && isdigit(fileName[i1]));
            const int number = ++i1 == i2 ? incr : atoi(fileName.substr(i1, i2 - i1).c_str()) + incr;
            const int width1 = i2 - i1, width2 = int(log10f(float(number)));
            const int width = std::max(width1, width2);

            char buf[10];
            switch(width)
            {
            case 2:     sprintf_s(buf, "%.2d", number);   break;
            case 3:     sprintf_s(buf, "%.3d", number);   break;
            case 4:     sprintf_s(buf, "%.4d", number);   break;
            case 5:     sprintf_s(buf, "%.5d", number);   break;
            case 6:     sprintf_s(buf, "%.6d", number);   break;
            case 7:     sprintf_s(buf, "%.7d", number);   break;
            case 8:     sprintf_s(buf, "%.8d", number);   break;
            case 9:     sprintf_s(buf, "%.9d", number);   break;
            case 10:    sprintf_s(buf, "%.10d", number);  break;
            default:    sprintf_s(buf, "%d", number);     break;
            }
            return fileName.substr(0, i1) + buf + fileName.substr(i2, len - i2);
        }
    }

}
#endif
