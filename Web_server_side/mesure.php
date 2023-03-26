<!DOCTYPE html>
<html>
<head>
    <meta http-equiv="refresh" content="300"> <!-- Actualiser toutes les 5 minutes (300 secondes) -->
</head>
<body>
<?php
ini_set('display_errors', 1);
error_reporting(E_ALL);
// Ouvre le fichier texte en mode lecture seule
$fichier = fopen("donnes.txt", "r");

// Aller à la fin du fichier
fseek($fichier, 0, SEEK_END);

// Récupérer la position de la dernière ligne
$position = ftell($fichier);

// Parcourir le fichier en partant de la fin jusqu'à trouver la deuxième ligne
$deuxieme_ligne = '';
while ($position > 0) {
    // Reculer d'une ligne
    $position--;

    // Se déplacer à la nouvelle position
    fseek($fichier, $position);

    // Lire le prochain caractère
    $caractere = fgetc($fichier);

    // Si on trouve deux sauts de ligne, on a trouvé la deuxième ligne à partir de la fin
    if ($caractere == "\n") {
        $ligne = fgets($fichier);
        if (!empty(trim($ligne))) {
            $deuxieme_ligne = $ligne;
            break;
        }
    }
}

// Fermer le fichier
fclose($fichier);

// Séparer les valeurs de la ligne en un tableau
$donnes = explode(',', $deuxieme_ligne);


// Affiche les données de la dernière ligne
echo "Mesure: " . $donnes[0] . " mm<br>";
echo "Température: " . $donnes[1] . " °C<br>";
echo "Voltage: " . $donnes[2] . " V<br>";
echo "identifiant de la sonde: " . $donnes[6] . " <br>";
echo "numerot de packet: " . $donnes[7] . " <br>";
echo "force signal radio : " . $donnes[8] . " dbm<br>";
echo "Date: " . $donnes[10] . "<br>";
echo "Heure: " . $donnes[11] . "<br>";
?>
</body>
</html>
