/* Plotter.h 
    * Created by Dhonan Nabil Hibatullah, Alfarobi v12
    * dhonan.hibatullah@gmail.com, open for any questions
    * Plotter.h is a library to plot quickly and seamlessly via C++ program
    * This library is based on gnuplot, which you can install with:
    * sudo apt install gnuplot (on linux)
*/   

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

            bool generateDat(std::vector<double> x_val, std::vector<double> y_val, std::string plot_name) {
                /* Generate .dat file */
                std::string file_name = dir_path + plot_name + (std::string)".dat";
                std::ofstream file(file_name);
                plot_files.push_back(file_name);

                /* Write the information inside */
                if(file.is_open()) {
                    for(uint64_t i = 0; i < x_val.size(); ++i) {
                        file << x_val[i] << ' ' << y_val[i] << std::endl;
                    }
                    file.close();
                    return true;
                }
                else {
                    ROS_ERROR("Failed to create %s.dat file", file_name.c_str());
                    return false;
                }
            }



        public:

            /* GnuPlotConfig structure
                * This struct is going to tell gnuplot how the plot will look like
                * we can directly pass it to the plot() function
            */
            struct plotconfig {
                std::string     label       = "plot",
                                line_color  = "blue",
                                plot_style  = PLOT_STYLE_INTERPOLATE;
            };



            Plotter(uint16_t rows, uint16_t cols) {
                /* Create alfarobi/plot directory if it not exists */
                struct stat dir;
                if(!(stat(dir_path.c_str(), &dir) == 0 && S_ISDIR(dir.st_mode))) {
                    if(!(mkdir(dir_path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == 0)) {
                        ROS_ERROR("Error while creating alfarobi_lib/plot");
                    }
                    else {
                        ROS_INFO("Directory alfarobi_lib/plot created!");
                    }
                }

                /* Create rows and cols */
                this->rows  = rows;
                this->cols  = cols;
                num_entry   = rows*cols;
                for(uint16_t i = 0; i < rows; ++i) {
                    std::vector<bool> init_status;
                    for(uint16_t j = 0; j < cols; ++j) {
                        titles.push_back((std::string)"Plot " + std::to_string(1 + i*cols + j));
                        x_labels.push_back("x");
                        y_labels.push_back("y");
                        cmds.push_back(" ");
                        init_status.push_back(false);
                    }
                    entry_status.push_back(init_status);
                }

                /* Determine the display aspects */
                size_x      = 1./(double)cols;
                size_y      = 1./(double)rows;

                /* Setup commands */
                setups.push_back(
                    (std::string)"set multiplot layout " + 
                    std::to_string(rows) + (std::string)", " +
                    std::to_string(cols) + (std::string)"\n"
                );
                for(uint16_t i = 0; i < rows; ++i) {
                    for(uint16_t j = 0; j < cols; ++j) {
                        setups.push_back(
                            (std::string)"set origin " +
                            std::to_string(((double)j)*size_x) + (std::string)", " +
                            std::to_string(((double)(rows - 1 - i))*size_y) + (std::string)"\n" +

                            (std::string)"set size " +
                            std::to_string(size_x) + (std::string)", " +
                            std::to_string(size_y) + (std::string)"\n"
                        );
                    }
                }
            }



            ~Plotter() {}



            /* setTitle() function
                * Set the x label of each plot
            */
            void setTitle(uint16_t row, uint16_t col, std::string str) {
                titles[row*cols + col] = str;
            }



            /* setXLabel() function
                * Set the x label of each plot
            */
            void setXLabel(uint16_t row, uint16_t col, std::string str) {
                x_labels[row*cols + col] = str;
            }



            /* setYLabel() function
                * Set the y label of each plot
            */
            void setYLabel(uint16_t row, uint16_t col, std::string str) {
                y_labels[row*cols + col] = str;
            }



            /* plot() function
                * This plot() takes two input, the x and y values
                * The x and y value must be in the same size as std::vector<double>
                * This function generates a .dat file inside alfarobi_lib/plot
            */
            void plot(std::vector<double> x_val, std::vector<double> y_val, plotconfig config, uint16_t row=0, uint16_t col=0) {
                generateDat(x_val, y_val, config.label);

                /* Save the plot comms */
                if(config.plot_style == PLOT_STYLE_INTERPOLATE) {
                    cmds[row*cols + col] =  cmds[row*cols + col] + 
                                            ((cmds[row*cols + col] == " ") ? (std::string)"" : (std::string)", ") +
                                            (std::string)"'" + plot_files.back() + (std::string)"'" +
                                            " with " + config.plot_style +
                                            " linecolor " + (std::string)"\"" + config.line_color + (std::string)"\"" +
                                            " title " + (std::string)"'" + config.label + (std::string)"'";
                }

                else if(config.plot_style == PLOT_STYLE_IMPULSE) {
                    cmds[row*cols + col] =  cmds[row*cols + col] + 
                                            ((cmds[row*cols + col] == " ") ? (std::string)"" : (std::string)", ") +
                                            (std::string)"'" + plot_files.back() + (std::string)"'" +
                                            " with " + config.plot_style;
                }

                else if(config.plot_style == PLOT_STYLE_POINT) {
                    cmds[row*cols + col] =  cmds[row*cols + col] + 
                                            ((cmds[row*cols + col] == " ") ? (std::string)"" : (std::string)", ") +
                                            (std::string)"'" + plot_files.back() + (std::string)"'" +
                                            " with " + config.plot_style;
                }

                else if(config.plot_style == PLOT_STYLE_HISTOGRAM) {
                    cmds[row*cols + col] =  cmds[row*cols + col] + 
                                            ((cmds[row*cols + col] == " ") ? (std::string)"" : (std::string)", ") +
                                            (std::string)"'" + plot_files.back() + (std::string)"'" +
                                            " with " + config.plot_style;
                }

                /* Set status to true */
                if(!entry_status[row][col]) num_plot += 1;
                entry_status[row][col] = true;
            }



            /* show() function
                * show() is a blocking function that will display the plot result
                * You can, however, make it non-blocking via threading
            */
            void show() {
                if(num_plot == num_entry) {
                    /* Open gnuplot pipe */
                    FILE* gnuplot_pipe = popen("gnuplot -persistent", "w");
                    if(!gnuplot_pipe) {
                        ROS_ERROR("Failed to open gnuplot!");
                        return;
                    }
                    
                    /* Set the window's title */
                    fprintf(gnuplot_pipe, "set term qt title %s", ((std::string)"\"" + main_title + (std::string)"\"\n").c_str());

                    /* Setup for multiplot layout */
                    fprintf(gnuplot_pipe, "%s", setups[0].c_str());

                    /* Display for each plot layouts */
                    for(uint16_t entry = 0; entry < num_entry; ++entry) {
                        /* Setups */
                        fprintf(
                            gnuplot_pipe, "%s", 
                            setups[entry + 1].c_str()
                        );

                        /* Title, x, and y label */
                        fprintf(
                            gnuplot_pipe, "%s", 
                            ((std::string) "set title " + (std::string)"\"" + titles[entry] + (std::string)"\"\n").c_str()    
                        );
                        fprintf(
                            gnuplot_pipe, "%s", 
                            ((std::string) "set xlabel " + (std::string)"\"" + x_labels[entry] + (std::string)"\"\n").c_str()    
                        );
                        fprintf(
                            gnuplot_pipe, "%s", 
                            ((std::string) "set ylabel " + (std::string)"\"" + y_labels[entry] + (std::string)"\"\n").c_str()    
                        );

                        /* Plot */
                        fprintf(
                            gnuplot_pipe, "%s",
                            ((std::string)"plot" + cmds[entry] + (std::string)"\n").c_str()
                        );
                    }

                    /* Close the pipe */
                    fprintf(gnuplot_pipe, "unset multiplot\n");
                    pclose(gnuplot_pipe);
                }

                else {
                    ROS_ERROR("There's %d plot slot(s) that needs to be filled!", num_entry - num_plot);
                }
            }
    };
}