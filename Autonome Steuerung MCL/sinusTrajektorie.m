function x = sinusTrajektorie(t_end, dt, amplitude, frequency, velocity)
% sinusTrajektorie berechnet eine Sinustrajektorie für ein Fahrzeug.
% 
% Eingaben:
%   - t_end: Endzeit der Trajektorie (Sekunden)
%   - dt: Zeitauflösung (Sekunden)
%   - amplitude: Amplitude der Sinusbewegung (Meter)
%   - frequency: Frequenz der Sinusbewegung (Hertz)
%   - velocity: Konstante Geschwindigkeit des Fahrzeugs in x-Richtung (m/s)
%
% Ausgang:
%   - x: Matrix mit [t, x, y, psi, xdot, ydot, psidot] als Spalten

    % Zeitvektor definieren
    t = 0:dt:t_end; % Zeitwerte in Sekunden
    
    % Positionen berechnen
    x = velocity * t;                  % x-Koordinaten (gleichförmige Bewegung)
    y = amplitude * sin(2 * pi * frequency * t); % y-Koordinaten (Sinusbewegung)

    % Ableitungen berechnen
    xdot = velocity * ones(size(t));  % x-Geschwindigkeit (konstant)
    ydot = 2 * pi * frequency * amplitude * cos(2 * pi * frequency * t); % y-Geschwindigkeit

    % Fahrzeugwinkel psi berechnen
    psi = atan2(ydot, xdot);          % Fahrtrichtung (Winkel in Rad)

    % Winkelgeschwindigkeit psidot berechnen
    psidot = (2 * pi * frequency * amplitude * (-sin(2 * pi * frequency * t))) ./ (sqrt(xdot.^2 + ydot.^2));

    % Ausgabe als Matrix zusammenfassen
    x = [t', x', y', psi', xdot', ydot', psidot'];
end