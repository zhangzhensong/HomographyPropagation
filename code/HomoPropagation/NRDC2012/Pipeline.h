#pragma once
#include <string>

class Pipeline
{
public:
    Pipeline(void);
    ~Pipeline(void);

    static void		TestBidirectional				(
        std::string work_dir_path,
        std::string forward_dir_path,
        std::string backward_dir_path);

    static void		TestReadCorre					(
        std::string work_dir_path,
        std::string forward_dir_path,
        std::string backward_dir_path
        );

    static void    TestTriangle();
};

