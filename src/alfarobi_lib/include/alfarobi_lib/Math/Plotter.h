/* Plotter.h 
    * Created by Dhonan Nabil Hibatullah, Alfarobi v12
    * dhonan.hibatullah@gmail.com, open for any questions
    * Plotter.h is a library to plot quickly and seamlessly via C++ program
    * This library is based on gnuplot, which you can install with:
    * sudo apt install gnuplot (on linux)
*/   

#ifndef PLOTTER_H
#define PLOTTER_H

#define NEGLIGIBLE_VAL      0.01
#define _USE_MATH_DEFINES

#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <cstdio>
#include <string>
#include <cmath>
#include <vector>
#include <ros/ros.h>
#include <ros/package.h>

/* Plot styles*/
#define PLOT_STYLE_INTERPOLATE   (std::string)"lines"
#define PLOT_STYLE_IMPULSE       (std::string)"impulses"
#define PLOT_STYLE_POINT         (std::string)"points"
#define PLOT_STYLE_HISTOGRAM     (std::string)"histograms"




namespace alfarobi {



    /* Plotter class
        * Declare this class to help you interface with gnuplot
    */
    class Plotter {
        
        

        private:

            std::string                     dir_path = ros::package::getPath("alfarobi_lib") + (std::string)"/plot/",
                                            main_title = "alfarobi::Plotter";

            std::vector<std::string>        plot_files,
                                            setups,
                                            cmds,
                                            x_labels,
                                            y_labels,
                                            titles;

            std::vector<std::vector<bool>>  entry_status;

            uint16_t                        rows,
                                            cols,
                                            num_entry,
                                            num_plot;

            double                          size_x,
                                            size_y;
        
        

        protected:

            bool generateDat(std::vector<double> x_val, std::vector<double> y_val, std::string plot_name);



        public:

            /* PlotConfig structure
                * This struct is going to tell gnuplot how the plot will look like
                * we can directly pass it to the plot() function
            */
            struct PlotConfig {
                std::string     label       = "plot",
                                line_color  = "blue",
                                plot_style  = PLOT_STYLE_INTERPOLATE;
            };


            Plotter(uint16_t rows, uint16_t cols);
            ~Plotter();

            void setTitle(uint16_t row, uint16_t col, std::string str);
            void setXLabel(uint16_t row, uint16_t col, std::string str);
            void setYLabel(uint16_t row, uint16_t col, std::string str);

            void plot(std::vector<double> x_val, std::vector<double> y_val, PlotConfig config, uint16_t row, uint16_t col);
            void show();
    };
}


#endif