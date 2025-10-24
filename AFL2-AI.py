graph = {
    'Indonesia': {'German': 200, 'America': 150, 'Singapore': 30},
    'German': {'Indonesia': 200, 'America': 230, 'Italy': 40, 'Brazil': 250},
    'America': {'German': 230, 'Indonesia': 150, 'Singapore': 100, 'Spain': 60, 'Japan': 80, 'Italy': 95},
    'Singapore': {'Indonesia': 30, 'America': 100, 'Spain': 75},
    'Spain': {'Singapore': 75, 'America': 60, 'Japan': 90},
    'Japan': {'Spain': 90, 'America': 80, 'Atlantis': 110},
    'Atlantis': {'Japan': 110, 'America': 100, 'Italy': 70, 'Africa': 170},
    'Italy': {'German': 40, 'Brazil': 215, 'Korea': 240, 'Africa': 140, 'Atlantis': 70, 'America': 95},
    'Brazil': {'German': 250, 'Italy': 215, 'Korea': 95},
    'Korea': {'Brazil': 95, 'Italy': 240, 'Africa': 180},
    'Africa': {'Korea': 180, 'Italy': 140, 'Atlantis': 170}
}

#Coordinates for heuristic
positions = {
    'Indonesia': (0, 3),
    'Singapore': (1, 1),
    'Spain': (2, 0),
    'America': (2, 2),
    'German': (2, 4),
    'Italy': (4, 3),
    'Brazil': (4, 5),
    'Korea': (6, 4),
    'Africa': (6, 2),
    'Atlantis': (5, 1),
    'Japan': (4, 0)
}

nodes = list(graph.keys())
