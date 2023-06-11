#include <vector>
#include <alfarobi_lib/Math/Plotter.h>


int main() {

    /* Create plotter class */
    alfarobi::Plotter plt = alfarobi::Plotter(2, 2);
    alfarobi::Plotter::PlotConfig cfg;



    /* Plot 1 */
    plt.setTitle(0, 0, "f_1 vs f_2 Plot");

    std::vector<double> x_val1 = {0, 1, 2, 3, 4, 5, 6};
    std::vector<double> y_val1 = {0, 1, 3, 1, -2, 1, 3};
    cfg.label = "f_1";
    cfg.line_color = "red";
    cfg.plot_style = PLOT_STYLE_INTERPOLATE;
    plt.plot(x_val1, y_val1, cfg, 0, 0);

    std::vector<double> x_val2 = {0, 1, 2, 3, 4, 5, 6};
    std::vector<double> y_val2 = {0, 2, 5, -2, -4, 0, -1};   
    cfg.label = "f_2";
    cfg.line_color = "blue";
    cfg.plot_style = PLOT_STYLE_INTERPOLATE;
    plt.plot(x_val2, y_val2, cfg, 0, 0);



    /* Plot 3 */
    plt.setTitle(0, 1, "f_3 Plot");

    std::vector<double> x_val3 = {0, 1, 2, 3, 4, 5, 6};
    std::vector<double> y_val3 = {0, 2, 5, -2, -4, 0, -1};   
    cfg.label = "f_3";
    cfg.line_color = "purple";
    cfg.plot_style = PLOT_STYLE_INTERPOLATE;
    plt.plot(x_val3, y_val3, cfg, 0, 1);



    /* Plot 4 */
    plt.setTitle(1, 0, "f_4 Plot");

    std::vector<double> x_val4 = {0, 1, 2, 3, 4, 5, 6};
    std::vector<double> y_val4 = {0, 2, 5, -2, -4, 0, -1};   
    cfg.label = "f_4";
    cfg.line_color = "brown";
    cfg.plot_style = PLOT_STYLE_INTERPOLATE;
    plt.plot(x_val4, y_val4, cfg, 1, 0);



    /* Plot 5 */
    plt.setTitle(1, 1, "f_5 Plot");

    std::vector<double> x_val5 = {0, 1, 2, 3, 4, 5, 6};
    std::vector<double> y_val5 = {0, 2, 5, -2, -4, 0, -1};   
    cfg.label = "f_5";
    cfg.line_color = "green";
    cfg.plot_style = PLOT_STYLE_INTERPOLATE;
    plt.plot(x_val5, y_val5, cfg, 1, 1);



    /* Show the plot */
    plt.show();
}