function Plot_States(tspan,state_vec,ref_input_vec,subplot_title)
    figure();
    subplot(221)
    plot(tspan./3600,state_vec(:,1),'LineWidth',2)
    grid on; grid minor
    xlabel('Time [hr]')
    ylabel('$\delta r$ [km]','Interpreter','latex')
    title(sprintf('x_1(t) Response for %s', ref_input_vec))
    subplot(223)
    plot(tspan./3600,state_vec(:,2),'LineWidth',2)
    grid on; grid minor
    xlabel('Time [hr]')
    ylabel('$\delta \dot{r}$ [km/s]','Interpreter','latex')
    title(sprintf('x_2(t) Response for %s', ref_input_vec))
    subplot(222)
    plot(tspan./3600,state_vec(:,3),'LineWidth',2)
    grid on; grid minor
    xlabel('Time [hr]')
    ylabel('$\delta \theta$ [rad]','Interpreter','latex')
    title(sprintf('x_3(t) Response for %s', ref_input_vec))
    subplot(224)
    plot(tspan./3600,state_vec(:,4),'LineWidth',2)
    grid on; grid minor
    xlabel('Time [hr]')
    ylabel('$\delta \dot{\theta}$ [rad/s]','Interpreter','latex')
    title(sprintf('x_4(t) Response for %s', ref_input_vec))
    sgtitle(subplot_title)
end

