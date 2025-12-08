function Plot_Outputs(tspan,output_vecs,ref_inputs_vecs,subplot_title)
    figure();
    subplot(221)
    plot(tspan./3600,output_vecs(:,1),'LineWidth',2)
    hold on; grid on; grid minor
    plot(tspan./3600,ref_inputs_vecs(:,1),'--k','LineWidth',2)
    yline(0.2*1.1,'-.','10% Overshoot','LabelVerticalAlignment','top',...
        'LineWidth',2)
    yline(0.2*1.02,':','2% Settling Time','LabelVerticalAlignment','top',...
        'LineWidth',2)
    yline(0.2*0.98,':','LineWidth',2)
    yline(0.2*0.95,'-.','95% Desired Reference','LabelVerticalAlignment',...
        'bottom','LineWidth',2)
    xline((10680+3600)/3600,'-.','1 Hour After Step Input',...
        'LabelVerticalAlignment','bottom','LineWidth',2)
    xline((10680+5400)/3600,':','1.5 Hours After Step Input',...
        'LabelVerticalAlignment','bottom','LineWidth',2)
    xlabel('Time [hr]')
    ylabel('$\delta r$ [km]','Interpreter','latex')
    title('y_1(t) Response for r_1(t)')
    subplot(223)
    plot(tspan./3600,wrapToPi(output_vecs(:,2)),'LineWidth',2)
    hold on; grid on; grid minor
    plot(tspan./3600,ref_inputs_vecs(:,2),'--k','LineWidth',2)
    xlabel('Time [hr]')
    ylabel('$\delta \theta$ [rad]','Interpreter','latex')
    title('y_2(t) Response for r_1(t)')
    subplot(222)
    plot(tspan./3600,output_vecs(:,3),'LineWidth',2)
    hold on; grid on; grid minor
    plot(tspan./3600,ref_inputs_vecs(:,4),'--k','LineWidth',2)
    xlabel('Time [hr]')
    ylabel('$\delta r$ [km/s]','Interpreter','latex')
    title('y_1(t) Response for r_2(t)')
    subplot(224)
    plot(tspan./3600,wrapToPi(output_vecs(:,4)),'LineWidth',2)
    hold on; grid on; grid minor
    plot(tspan./3600,ref_inputs_vecs(:,5),'--k','LineWidth',2)
    yline(1e-4*1.1,'-.','10% Overshoot','LabelVerticalAlignment',...
        'top','LineWidth',2)
    yline(1e-4*1.02,':','2% Settling Time','LabelVerticalAlignment',...
        'top','LineWidth',2)
    yline(1e-4*0.98,':','LineWidth',2)
    yline(1e-4*0.95,'-.','95% Desired Reference','LabelVerticalAlignment',...
        'bottom','LineWidth',2)
    xline((10680+3600)/3600,'-.','1 Hour After Step Input',...
        'LabelVerticalAlignment','bottom','LineWidth',2)
    xline((10680+5400)/3600,':','1.5 Hours After Step Input',...
        'LabelVerticalAlignment','bottom','LineWidth',2)
    xlabel('Time [hr]')
    ylabel('$\delta \theta$ [rad]','Interpreter','latex')
    title('y_2(t) Response for r_2(t)')
    sgtitle(subplot_title)
end


