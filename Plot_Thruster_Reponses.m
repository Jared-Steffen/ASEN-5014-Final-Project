function Plot_Thruster_Reponses(tspan,u_vec,u_max,subplot_title)
    figure();
    subplot(121)
    plot(tspan,u_vec(:,1),'LineWidth',2)
    hold on; grid on; grid minor
    plot(tspan,u_vec(:,2),'LineWidth',2)
    plot(tspan,u_max*ones(length(tspan)),'k:','LineWidth',2)
    plot(tspan,-u_max*ones(length(tspan)),'k:','LineWidth',2)
    xlabel('Time [s]')
    ylabel('Radial Thruster \delta u_{1,2} [km/s^2]')
    title('Thruster Response to r_1(t)')
    subplot(122)
    plot(tspan,u_vec(:,3),'LineWidth',2)
    hold on; grid on; grid minor
    plot(tspan,u_vec(:,4),'LineWidth',2)
    plot(tspan,u_max*ones(length(tspan)),'k:','LineWidth',2)
    plot(tspan,-u_max*ones(length(tspan)),'k:','LineWidth',2)
    xlabel('Time [s]')
    ylabel('Tangential Thruster \delta u_{1,2} [km/s^2]')
    title('Thruster Response to r_2(t)')
    legend('u_1(t)','u_2(t)','u_{max}')
    sgtitle(subplot_title)
end

