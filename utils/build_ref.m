% ========== Funcion Referencia ==================
function [t_ref, ref, dref, ddref] = build_ref(dt, Tf, tipo_trayectoria)
% build_ref: Genera la referencia de posición, velocidad y aceleración.
% Entradas:
%   dt : Paso de tiempo
%   Tf : Tiempo final
%   tipo_trayectoria: 1 para Círculo, 2 para Lemniscata

    % Vector de tiempo base
    t_ref = 0:dt:Tf+2*dt; % Agregamos 2 pasos extra para evitar problemas de índice en el control
    N = length(t_ref);
    
    % Inicialización de matrices (3 filas para X, Y y Z, N columnas para el tiempo)
    ref   = zeros(3, N);
    dref  = zeros(3, N);
    ddref = zeros(3, N);

    switch tipo_trayectoria
        case 1 % ===== OPCIÓN: TRAYECTORIA CONSTANTE =====
            ref(1, :) = 5.0 * ones(1, N); % Queremos ir a X = 5 metros
            ref(2, :) = 5.0 * ones(1, N); % Queremos ir a Y = 5 metros
            ref(3, :) = 6 + 0.5*t_ref;      % Queremos mantener Z constante

            % Velocidad (Derivada numérica)
            % Calculamos la diferencia entre puntos y dividimos por dt
            dref(1, 1:end-1) = diff(ref(1, :)) / dt;
            dref(2, 1:end-1) = diff(ref(2, :)) / dt;
            dref(3, 1:end-1) = diff(ref(3, :)) / dt;
            % El último valor lo igualamos al anterior para mantener el tamaño
            dref(:, end) = dref(:, end-1);
            
            % Aceleración (Derivada numérica de la velocidad)
            ddref(1, 1:end-1) = diff(dref(1, :)) / dt;
            ddref(2, 1:end-1) = diff(dref(2, :)) / dt;
            ddref(3, 1:end-1) = diff(dref(3, :)) / dt;
            % El último valor lo igualamos al anterior para mantener el tamaño
            ddref(:, end) = ddref(:, end-1);

        case 2 % ===== OPCIÓN: CÍRCULO =====
            Radio = 10.0;
            Frecuencia = 0.05;
            Omega = 2 * pi * Frecuencia;

            ref(1, :) = Radio * cos(Omega * t_ref);
            ref(2, :) = Radio * sin(Omega * t_ref);
            ref(3, :) = 6 + 0.5*t_ref; % Referencia de Altura (Rampa)

            % Velocidad (Derivada numérica)
            % Calculamos la diferencia entre puntos y dividimos por dt
            dref(1, 1:end-1) = diff(ref(1, :)) / dt;
            dref(2, 1:end-1) = diff(ref(2, :)) / dt;
            dref(3, 1:end-1) = diff(ref(3, :)) / dt;
            % El último valor lo igualamos al anterior para mantener el tamaño
            dref(:, end) = dref(:, end-1);
            
            % Aceleración (Derivada numérica de la velocidad)
            ddref(1, 1:end-1) = diff(dref(1, :)) / dt;
            ddref(2, 1:end-1) = diff(dref(2, :)) / dt;
            ddref(3, 1:end-1) = diff(dref(3, :)) / dt;
            % El último valor lo igualamos al anterior para mantener el tamaño
            ddref(:, end) = ddref(:, end-1);
            
            %{
            dref(1, :) = -Radio * Omega * sin(Omega * t_ref);
            dref(2, :) =  Radio * Omega * cos(Omega * t_ref);
            dref(3, :) = 0.5 * ones(1, N);
            %}
            
            %{
            ddref(1, :) = -Radio * (Omega^2) * cos(Omega * t_ref);
            ddref(2, :) = -Radio * (Omega^2) * sin(Omega * t_ref);
            ddref(3, :) = zeros(1, N);
            %}

        case 3 % ===== OPCIÓN: LEMNISCATA DE BERNOULLI =====
            a_lem = 10;        % Amplitud (ajustada a 10m como el círculo)
            omega_lem = 0.3;   % Velocidad angular
            
            % Ecuación paramétrica
            den = 1 + sin(omega_lem * t_ref).^2;
            ref(1, :) = (a_lem * cos(omega_lem * t_ref)) ./ den;
            ref(2, :) = (a_lem * sin(omega_lem * t_ref) .* cos(omega_lem * t_ref)) ./ den;
            ref(3, :) = 6 + 0.5*t_ref; % Referencia de Altura (Rampa)

            % Velocidad (Derivada numérica)
            % Calculamos la diferencia entre puntos y dividimos por dt
            dref(1, 1:end-1) = diff(ref(1, :)) / dt;
            dref(2, 1:end-1) = diff(ref(2, :)) / dt;
            dref(3, 1:end-1) = diff(ref(3, :)) / dt;
            % El último valor lo igualamos al anterior para mantener el tamaño
            dref(:, end) = dref(:, end-1);
            
            % Aceleración (Derivada numérica de la velocidad)
            ddref(1, 1:end-1) = diff(dref(1, :)) / dt;
            ddref(2, 1:end-1) = diff(dref(2, :)) / dt;
            ddref(3, 1:end-1) = diff(dref(3, :)) / dt;
            % El último valor lo igualamos al anterior para mantener el tamaño
            ddref(:, end) = ddref(:, end-1);
    end
end