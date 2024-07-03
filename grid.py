import pygame
import sys
import xml.etree.ElementTree as ET

# Initialisation de Pygame
pygame.init()

# Vérification des arguments de ligne de commande
if len(sys.argv) != 2:
    print("Usage: python3 grid.py <taille_grille>")
    sys.exit(1)

# Définition des constantes
TAILLE_GRILLE = int(sys.argv[1])
if TAILLE_GRILLE % 2 == 0:
    print("La taille de la grille doit être un nombre impair.")
    sys.exit(1)

TAILLE_CARRE = 30
LARGEUR_FENETRE = TAILLE_GRILLE * TAILLE_CARRE
HAUTEUR_FENETRE = TAILLE_GRILLE * TAILLE_CARRE + 50  # Espace supplémentaire pour le bouton
COULEUR_FOND = (255, 255, 255)
COULEUR_LIGNE = (0, 0, 0)
COULEUR_CARRE = (255, 255, 255)
COULEUR_CENTRE = (0, 0, 255)  # Bleu pour la case centrale
COULEUR_SORTIE = (255, 0, 0)  # Rouge pour la case de sortie
COULEUR_BOUTON = (200, 200, 200)
COULEUR_TEXTE = (0, 0, 0)

# Compteur pour les fichiers sauvegardés
compteur = 1

# Création de la fenêtre
fenetre = pygame.display.set_mode((LARGEUR_FENETRE, HAUTEUR_FENETRE))
pygame.display.set_caption(f"Grille {TAILLE_GRILLE}x{TAILLE_GRILLE}")

# Création de la grille
grille = [[COULEUR_CARRE for _ in range(TAILLE_GRILLE)] for _ in range(TAILLE_GRILLE)]
centre_x, centre_y = TAILLE_GRILLE // 2, TAILLE_GRILLE // 2
grille[centre_y][centre_x] = COULEUR_CENTRE

# Définition du bouton "Save"
BOUTON_RECT = pygame.Rect((LARGEUR_FENETRE // 2 - 50, HAUTEUR_FENETRE - 40, 100, 30))

def dessiner_grille():
    for y in range(TAILLE_GRILLE):
        for x in range(TAILLE_GRILLE):
            rect = pygame.Rect(x * TAILLE_CARRE, y * TAILLE_CARRE, TAILLE_CARRE, TAILLE_CARRE)
            pygame.draw.rect(fenetre, grille[y][x], rect)
            pygame.draw.rect(fenetre, COULEUR_LIGNE, rect, 1)

def dessiner_bouton():
    pygame.draw.rect(fenetre, COULEUR_BOUTON, BOUTON_RECT)
    font = pygame.font.SysFont(None, 24)
    texte = font.render("Save", True, COULEUR_TEXTE)
    fenetre.blit(texte, (BOUTON_RECT.x + 25, BOUTON_RECT.y + 5))

def changer_couleur(x, y, bouton):
    if 0 <= x < TAILLE_GRILLE and 0 <= y < TAILLE_GRILLE:  # Vérification des coordonnées
        if (x, y) != (centre_x, centre_y):  # Vérifier que ce n'est pas la case centrale
            if bouton == 1:  # Clic gauche
                if grille[y][x] == COULEUR_CARRE:
                    grille[y][x] = COULEUR_LIGNE
                elif grille[y][x] == COULEUR_LIGNE:
                    grille[y][x] = COULEUR_CARRE
            elif bouton == 3:  # Clic droit
                if grille[y][x] in (COULEUR_CARRE, COULEUR_LIGNE):
                    grille[y][x] = COULEUR_SORTIE
                elif grille[y][x] == COULEUR_SORTIE:
                    grille[y][x] = COULEUR_CARRE

def tourner_grille_horaire(grille):
    return [[grille[TAILLE_GRILLE - 1 - x][y] for x in range(TAILLE_GRILLE)] for y in range(TAILLE_GRILLE)]

def trouver_sortie(grille):
    for y in range(TAILLE_GRILLE):
        for x in range(TAILLE_GRILLE):
            if grille[y][x] == COULEUR_SORTIE:
                return x, y
    return None, None

def generer_xml():
    global compteur  # Utilisation de la variable globale pour le compteur
    grille_tournee = tourner_grille_horaire(grille)
    sortie_x, sortie_y = trouver_sortie(grille_tournee)

    sdf = ET.Element("sdf", version="1.6")
    world = ET.SubElement(sdf, "world", name="default")

    include_ground = ET.SubElement(world, "include")
    uri_ground = ET.SubElement(include_ground, "uri")
    uri_ground.text = "model://ground_plane"

    include_sun = ET.SubElement(world, "include")
    uri_sun = ET.SubElement(include_sun, "uri")
    uri_sun.text = "model://sun"

    scene = ET.SubElement(world, "scene")
    shadows = ET.SubElement(scene, "shadows")
    shadows.text = "false"

    gui = ET.SubElement(world, "gui", fullscreen='0')
    camera = ET.SubElement(gui, "camera", name='user_camera')
    pose = ET.SubElement(camera, "pose", frame='')
    pose.text = "0.319654 -0.235002 9.29441 0 1.5138 0.009599"
    view_controller = ET.SubElement(camera, "view_controller")
    view_controller.text = "orbit"
    projection_type = ET.SubElement(camera, "projection_type")
    projection_type.text = "perspective"

    physics = ET.SubElement(world, "physics", type="ode")
    real_time_update_rate = ET.SubElement(physics, "real_time_update_rate")
    real_time_update_rate.text = "1000.0"
    max_step_size = ET.SubElement(physics, "max_step_size")
    max_step_size.text = "0.001"
    real_time_factor = ET.SubElement(physics, "real_time_factor")
    real_time_factor.text = "1"
    ode = ET.SubElement(physics, "ode")
    solver = ET.SubElement(ode, "solver")
    solver_type = ET.SubElement(solver, "type")
    solver_type.text = "quick"
    iters = ET.SubElement(solver, "iters")
    iters.text = "150"
    precon_iters = ET.SubElement(solver, "precon_iters")
    precon_iters.text = "0"
    sor = ET.SubElement(solver, "sor")
    sor.text = "1.400000"
    use_dynamic_moi_rescaling = ET.SubElement(solver, "use_dynamic_moi_rescaling")
    use_dynamic_moi_rescaling.text = "1"
    constraints = ET.SubElement(ode, "constraints")
    cfm = ET.SubElement(constraints, "cfm")
    cfm.text = "0.00001"
    erp = ET.SubElement(constraints, "erp")
    erp.text = "0.2"
    contact_max_correcting_vel = ET.SubElement(constraints, "contact_max_correcting_vel")
    contact_max_correcting_vel.text = "2000.000000"
    contact_surface_layer = ET.SubElement(constraints, "contact_surface_layer")
    contact_surface_layer.text = "0.01000"

    block_counter = 1
    for y in range(TAILLE_GRILLE):
        for x in range(TAILLE_GRILLE):
            if grille_tournee[y][x] == COULEUR_LIGNE:
                model = ET.SubElement(world, "model", name=f"block_wall_{block_counter}")
                include = ET.SubElement(model, "include")
                pose = ET.SubElement(include, "pose")
                pose.text = f"{(x - TAILLE_GRILLE // 2) * 0.5} {(TAILLE_GRILLE // 2 - y) * 0.5} 0 0 0 0"
                uri = ET.SubElement(include, "uri")
                uri.text = "model://turtlebot3_drl_world/inner_block_wall"
                block_counter += 1

    include_turtlebot = ET.SubElement(world, "include")
    pose_turtlebot = ET.SubElement(include_turtlebot, "pose")
    pose_turtlebot.text = "0 0 0 0 0 0"
    uri_turtlebot = ET.SubElement(include_turtlebot, "uri")
    uri_turtlebot.text = "model://turtlebot3_burger"

    filename = f"maze_world_{compteur}.xml"
    tree = ET.ElementTree(sdf)
    
    # Ajouter un commentaire avec les coordonnées de la sortie
    if sortie_x is not None and sortie_y is not None:
        commentaire = ET.Comment(f"Maze exit: ({(sortie_x - TAILLE_GRILLE // 2) * 0.5}, {(TAILLE_GRILLE // 2 - sortie_y) * 0.5})")
        sdf.append(commentaire)
    
    tree.write(filename, encoding="utf-8", xml_declaration=True)
    print(f"saved as {filename}")
    compteur += 1  # Incrémenter le compteur pour le prochain fichier

# Variable pour suivre l'état du clic de la souris
clic_enfonce = False
dernier_bouton = None
derniere_case = (-1, -1)

# Boucle principale
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.MOUSEBUTTONDOWN:
            clic_enfonce = True
            dernier_bouton = event.button
            x, y = event.pos
            if BOUTON_RECT.collidepoint(x, y):
                generer_xml()
            else:
                grille_x = x // TAILLE_CARRE
                grille_y = y // TAILLE_CARRE
                changer_couleur(grille_x, grille_y, dernier_bouton)
                derniere_case = (grille_x, grille_y)
        elif event.type == pygame.MOUSEBUTTONUP:
            clic_enfonce = False
            dernier_bouton = None
            derniere_case = (-1, -1)
        elif event.type == pygame.MOUSEMOTION and clic_enfonce:
            x, y = event.pos
            if not BOUTON_RECT.collidepoint(x, y):
                grille_x = x // TAILLE_CARRE
                grille_y = y // TAILLE_CARRE
                if (grille_x, grille_y) != derniere_case:
                    changer_couleur(grille_x, grille_y, dernier_bouton)
                    derniere_case = (grille_x, grille_y)
    
    fenetre.fill(COULEUR_FOND)
    dessiner_grille()
    dessiner_bouton()
    pygame.display.flip()

pygame.quit()
sys.exit()
