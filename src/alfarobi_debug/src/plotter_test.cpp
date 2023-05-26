#include <alfarobi_lib/Math/Plotter.h>


int main() {

    /* Create plotter class */
    alfarobi::Plotter plt = alfarobi::Plotter(2, 2);
    alfarobi::Plotter::plotconfig conf;



    /* Plot 1 */
    plt.setTitle(0, 0, "f_1 vs f_2 Plot");

    std::vector<double> x_val1 = {0, 1, 2, 3, 4, 5, 6};
    std::vector<double> y_val1 = {0, 1, 3, 1, -2, 1, 3};
    conf.label = "f_1";
    conf.line_color = "red";
    conf.plot_style = PLOT_STYLE_INTERPOLATE;
    plt.plot(x_val1, y_val1, conf, 0, 0);

    std::vector<double> x_val2 = {0, 1, 2, 3, 4, 5, 6};
    std::vector<double> y_val2 = {0, 2, 5, -2, -4, 0, -1};   
    conf.label = "f_2";
    conf.line_color = "blue";
    conf.plot_style = PLOT_STYLE_INTERPOLATE;
    plt.plot(x_val2, y_val2, conf, 0, 0);



    /* Plot 3 */
    plt.setTitle(0, 1, "f_3 Plot");

    std::vector<double> x_val3 = {0, 1, 2, 3, 4, 5, 6};
    std::vector<double> y_val3 = {0, 2, 5, -2, -4, 0, -1};   
    conf.label = "f_3";
    conf.line_color = "purple";
    conf.plot_style = PLOT_STYLE_INTERPOLATE;
    plt.plot(x_val3, y_val3, conf, 0, 1);



    /* Plot 4 */
    plt.setTitle(1, 0, "f_4 Plot");

    std::vector<double> x_val4 = {0, 1, 2, 3, 4, 5, 6};
    std::vector<double> y_val4 = {0, 2, 5, -2, -4, 0, -1};   
    conf.label = "f_4";
    conf.line_color = "brown";
    conf.plot_style = PLOT_STYLE_INTERPOLATE;
    plt.plot(x_val4, y_val4, conf, 1, 0);



    /* Plot 5 */
    plt.setTitle(1, 1, "f_5 Plot");

    std::vector<double> x_val5 = {0, 1, 2, 3, 4, 5, 6};
    std::vector<double> y_val5 = {0, 2, 5, -2, -4, 0, -1};   
    conf.label = "f_5";
    conf.line_color = "green";
    conf.plot_style = PLOT_STYLE_INTERPOLATE;
    plt.plot(x_val5, y_val5, conf, 1, 1);



    /* Show the plot */
    plt.show();
}