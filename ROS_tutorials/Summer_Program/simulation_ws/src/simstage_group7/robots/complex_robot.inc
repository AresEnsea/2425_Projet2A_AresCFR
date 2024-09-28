define laser ranger(
	sensor(
		range [0.0 5.6]
		fov 240.0
		samples 240
	)

	color "blue"
	size [0.01 0.01 0.01]

	block(
		points 4
		point[0] [0 0]
		point[1] [0 1]
		point[2] [1 1]
		point[3] [1 0]
		z [0 1]
	)
)

define complex_robot position(
	odom_error [0.03 0.03 0.00 0.05]
	localization "odom"
	size [0.1 0.075 0.15]
	gui_nose 1
	drive "diff"
	color "red"
	
	
# 1ère couche
# 1ère couche
# 1ère couche

# 1ère couche
block(
    points 4
    point[0] [0.2 0.2]
    point[1] [0.2 0.8]
    point[2] [0.7 0.8]
    point[3] [0.7 0.2]
    z [0.1 0.2]
    color "red"
)

# 1ère couche
block(
    points 4
    point[0] [0.3 0.2]
    point[1] [0.3 0.8]
    point[2] [0.8 0.8]
    point[3] [0.8 0.2]
    z [0.1 0.2]
    color "red"
)

# 1ère couche
block(
    points 4
    point[0] [0.2 0.2]
    point[1] [0.2 0.7]
    point[2] [0.8 0.7]
    point[3] [0.8 0.2]
    z [0.1 0.2]
    color "red"
)

# 1ère couche
block(
    points 4
    point[0] [0.2 0.3]
    point[1] [0.2 0.8]
    point[2] [0.8 0.8]
    point[3] [0.8 0.3]
    z [0.1 0.2]
    color "red"
)

# 1ère couche
block(
    points 4
    point[0] [0.1 0.1]
    point[1] [0.1 0.9]
    point[2] [0.8 0.9]
    point[3] [0.8 0.1]
    z [0.1 0.2]
    color "red"
)

# 1ère couche
block(
    points 4
    point[0] [0.2 0.1]
    point[1] [0.2 0.9]
    point[2] [0.9 0.9]
    point[3] [0.9 0.1]
    z [0.1 0.2]
    color "red"
)

# 1ère couche
block(
    points 4
    point[0] [0.1 0.1]
    point[1] [0.1 0.8]
    point[2] [0.9 0.8]
    point[3] [0.9 0.1]
    z [0.1 0.2]
    color "red"
)

# 1ère couche
block(
    points 4
    point[0] [0.1 0.2]
    point[1] [0.1 0.9]
    point[2] [0.9 0.9]
    point[3] [0.9 0.2]
    z [0.1 0.2]
    color "red"
)

# 1ère couche
block(
    points 4
    point[0] [0 0]
    point[1] [0 1]
    point[2] [0.9 1]
    point[3] [0.9 0]
    z [0.1 0.2]
    color "red"
)

# 1ère couche
block(
    points 4
    point[0] [0.1 0]
    point[1] [0.1 1]
    point[2] [1 1]
    point[3] [1 0]
    z [0.1 0.2]
    color "red"
)

# 1ère couche
block(
    points 4
    point[0] [0 0]
    point[1] [0 0.9]
    point[2] [1 0.9]
    point[3] [1 0]
    z [0.1 0.2]
    color "red"
)

# 1ère couche
block(
    points 4
    point[0] [0 0.1]
    point[1] [0 1]
    point[2] [1 1]
    point[3] [1 0.1]
    z [0.1 0.2]
    color "red"
)

# 2ème couche
# 2ème couche
# 2ème couche

# 2ème couche
block(
    points 4
    point[0] [0.2 0.2]
    point[1] [0.2 0.8]
    point[2] [0.7 0.8]
    point[3] [0.7 0.2]
    z [0.5 0.6]
    color "red"
)

# 2ème couche
block(
    points 4
    point[0] [0.3 0.2]
    point[1] [0.3 0.8]
    point[2] [0.8 0.8]
    point[3] [0.8 0.2]
    z [0.5 0.6]
    color "red"
)# 2ème couche
block(
    points 4
    point[0] [0.2 0.2]
    point[1] [0.2 0.8]
    point[2] [0.7 0.8]
    point[3] [0.7 0.2]
    z [0.5 0.6]
    color "red"
)

# 2ème couche
block(
    points 4
    point[0] [0.3 0.2]
    point[1] [0.3 0.8]
    point[2] [0.8 0.8]
    point[3] [0.8 0.2]
    z [0.5 0.6]
    color "red"
)

# 2ème couche
block(
    points 4
    point[0] [0.1 0.1]
    point[1] [0.1 0.9]
    point[2] [0.8 0.9]
    point[3] [0.8 0.1]
    z [0.5 0.6]
    color "red"
)

# 2ème couche
block(
    points 4
    point[0] [0.2 0.1]
    point[1] [0.2 0.9]
    point[2] [0.9 0.9]
    point[3] [0.9 0.1]
    z [0.5 0.6]
    color "red"
)

# 2ème couche
block(
    points 4
    point[0] [0.1 0.1]
    point[1] [0.1 0.8]
    point[2] [0.9 0.8]
    point[3] [0.9 0.1]
    z [0.5 0.6]
    color "red"
)

# 2ème couche
block(
    points 4
    point[0] [0.1 0.2]
    point[1] [0.1 0.9]
    point[2] [0.9 0.9]
    point[3] [0.9 0.2]
    z [0.5 0.6]
    color "red"
)

# 2ème couche
block(
    points 4
    point[0] [0 0]
    point[1] [0 1]
    point[2] [0.9 1]
    point[3] [0.9 0]
    z [0.5 0.6]
    color "red"
)

# 2ème couche
block(
    points 4
    point[0] [0.1 0]
    point[1] [0.1 1]
    point[2] [1 1]
    point[3] [1 0]
    z [0.5 0.6]
    color "red"
)

# 2ème couche
block(
    points 4
    point[0] [0 0]
    point[1] [0 0.9]
    point[2] [1 0.9]
    point[3] [1 0]
    z [0.5 0.6]
    color "red"
)

# 2ème couche
block(
    points 4
    point[0] [0 0.1]
    point[1] [0 1]
    point[2] [1 1]
    point[3] [1 0.1]
    z [0.5 0.6]
    color "red"
)

# 3ème couche
# 3ème couche
# 3ème couche

# 3ème couche
block(
    points 4
    point[0] [0.2 0.2]
    point[1] [0.2 0.8]
    point[2] [0.7 0.8]
    point[3] [0.7 0.2]
    z [0.9 1]
    color "red"
)

# 3ème couche
block(
    points 4
    point[0] [0.3 0.2]
    point[1] [0.3 0.8]
    point[2] [0.8 0.8]
    point[3] [0.8 0.2]
    z [0.9 1]
    color "red"
)

# 3ème couche
block(
    points 4
    point[0] [0.2 0.2]
    point[1] [0.2 0.7]
    point[2] [0.8 0.7]
    point[3] [0.8 0.2]
    z [0.9 1]
    color "red"
)

# 3ème couche
block(
    points 4
    point[0] [0.2 0.3]
    point[1] [0.2 0.8]
    point[2] [0.8 0.8]
    point[3] [0.8 0.3]
    z [0.9 1]
    color "red"
)


# 3ème couche
block(
    points 4
    point[0] [0.1 0.1]
    point[1] [0.1 0.9]
    point[2] [0.8 0.9]
    point[3] [0.8 0.1]
    z [0.9 1]
    color "red"
)

# 3ème couche
block(
    points 4
    point[0] [0.2 0.1]
    point[1] [0.2 0.9]
    point[2] [0.9 0.9]
    point[3] [0.9 0.1]
    z [0.9 1]
    color "red"
)

# 3ème couche
block(
    points 4
    point[0] [0.1 0.1]
    point[1] [0.1 0.8]
    point[2] [0.9 0.8]
    point[3] [0.9 0.1]
    z [0.9 1]
    color "red"
)

# 3ème couche
block(
    points 4
    point[0] [0.1 0.2]
    point[1] [0.1 0.9]
    point[2] [0.9 0.9]
    point[3] [0.9 0.2]
    z [0.9 1]
    color "red"
)

# 3ème couche
block(
    points 4
    point[0] [0 0]
    point[1] [0 1]
    point[2] [0.9 1]
    point[3] [0.9 0]
    z [0.9 1]
    color "red"
)

# 3ème couche
block(
    points 4
    point[0] [0.1 0]
    point[1] [0.1 1]
    point[2] [1 1]
    point[3] [1 0]
    z [0.9 1]
    color "red"
)

# 3ème couche
block(
    points 4
    point[0] [0 0]
    point[1] [0 0.9]
    point[2] [1 0.9]
    point[3] [1 0]
    z [0.9 1]
    color "red"
)

# 3ème couche
block(
    points 4
    point[0] [0 0.1]
    point[1] [0 1]
    point[2] [1 1]
    point[3] [1 0.1]
    z [0.9 1]
    color "red"
)

# Roue gauche
block(
    points 4
    point[0] [0.3 0.2]  
    point[1] [0.3 0.3]
    point[2] [0.7 0.3]
    point[3] [0.7 0.2]
    z [0 0.1] 
    color "black"
)

# Roue droite
block(
    points 4
    point[0] [0.3 0.7]  
    point[1] [0.3 0.8]
    point[2] [0.7 0.8]
    point[3] [0.7 0.7]
    z [0 0.1]  
    color "black"
)


# COLONNE ETAGE INFERIEUR 
# COLONNE ETAGE INFERIEUR 
# COLONNE ETAGE INFERIEUR 


# colonne 1 (devant à gauche)
block(
    points 4
    point[0] [0.75 0.15]  
    point[1] [0.75 0.25]
    point[2] [0.85 0.25]
    point[3] [0.85 0.15]
    z [0.2 0.5]  
    color "black"
)


# colonne 2 (devant à droite)
block(
    points 4
    point[0] [0.75 0.85]  
    point[1] [0.75 0.75]
    point[2] [0.85 0.75]
    point[3] [0.85 0.85]
    z [0.2 0.5]  
    color "black"
)

# colonne 3 (au fond à droite)
block(
    points 4
    point[0] [0.25 0.85]  
    point[1] [0.25 0.75]
    point[2] [0.15 0.75]
    point[3] [0.15 0.85]
    z [0.2 0.5]  
    color "black"
)


# colonne 4 (au fond à gauche) 
block(
    points 4
    point[0] [0.25 0.15]  
    point[1] [0.25 0.25]
    point[2] [0.15 0.25]
    point[3] [0.15 0.15]
    z [0.2 0.5]  
    color "black"
)


# COLONNE ETAGE SUPERIEUR 
# COLONNE ETAGE SUPERIEUR 
# COLONNE ETAGE SUPERIEUR 

# colonne 1 (devant à gauche)
block(
    points 4
    point[0] [0.75 0.15]  
    point[1] [0.75 0.25]
    point[2] [0.85 0.25]
    point[3] [0.85 0.15]
    z [0.6 0.9]  
    color "black"
)


# colonne 2 (devant à droite)
block(
    points 4
    point[0] [0.75 0.85]  
    point[1] [0.75 0.75]
    point[2] [0.85 0.75]
    point[3] [0.85 0.85]
    z [0.6 0.9]  
    color "black"
)

# colonne 3 (au fond à droite)
block(
    points 4
    point[0] [0.25 0.85]  
    point[1] [0.25 0.75]
    point[2] [0.15 0.75]
    point[3] [0.15 0.85]
    z [0.6 0.9]  
    color "black"
)


# colonne 4 (au fond à gauche) 
block(
    points 4
    point[0] [0.25 0.15]  
    point[1] [0.25 0.25]
    point[2] [0.15 0.25]
    point[3] [0.15 0.15]
    z [0.6 0.9]  
    color "black"
)     
	laser(pose [0.055 0.0 -0.12 0.0])
)
