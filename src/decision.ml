(**

   Chers programmeuses et programmeurs de λman, votre mission consiste
   à compléter ce module pour faire de vos λmen les meilleurs robots
   de la galaxie. C'est d'ailleurs le seul module qui vous pouvez
   modifier pour mener à bien votre mission.

   La fonction à programmer est

         [decide : memory -> observation -> action * memory]

   Elle est appelée à chaque unité de temps par le Λserver avec de
   nouvelles observations sur son environnement. En réponse à ces
   observations, cette fonction décide quelle action doit effectuer le
   robot.

   L'état du robot est représenté par une valeur de type [memory].  La
   fonction [decide] l'attend en argument et renvoie une nouvelle
   version de cette mémoire. Cette nouvelle version sera passée en
   argument à [decide] lors du prochain appel.

*)

open World
open Space

(** Le Λserver transmet les observations suivantes au λman: *)
type observation = World.observation

(** Votre λman peut se déplacer : il a une direction D et une vitesse V.

    La direction est en radian dans le repère trigonométrique standard :

    - si D = 0. alors le robot pointe vers l'est.
    - si D = Float.pi /. 2.  alors le robot pointe vers le nord.
    - si D = Float.pi alors le robot pointe vers l'ouest.
    - si D = 3 * Float.pi / 2 alors le robot pointe vers le sud.
    (Bien entendu, ces égalités sont à lire "modulo 2 * Float.pi".)

    Son déplacement en abscisse est donc V * cos D * dt et en ordonnée
   V * sin D * dt.

    Votre λman peut communiquer : il peut laisser du microcode sur sa
   position courante pour que d'autres λmen puissent le lire.  Un
   microcode s'autodétruit au bout d'un certain nombre d'unités de
   temps mais si un microcode est laissé près d'un autre microcode
   identique, ils fusionnent en un unique microcode dont la durée de
   vie est le somme des durées de vie des deux microcodes initiaux.
   Construire un microcode demande de l'énergie au robot : chaque
   atome lui coûte 1 point d'énergie. Heureusement, l'énergie augmente
   d'1 point toutes les unités de temps.

    Pour terminer, votre λman peut couper des arbres de Böhm. Les
   arbres de Böhm ont un nombre de branches variables. Couper une
   branche prend une unité de temps et augmente le score de 1
   point. Si on ramène cette branche au vaisseau, un second point est
   accordé.

    Pour finir, le monde est malheureusement très dangereux : on y
   trouve des bouches de l'enfer dans lesquelles il ne faut pas tomber
   ainsi que des champs de souffrances où la vitesse de votre robot
   est modifiée (de -50% à +50%).

*)

type action =
  | Move of Space.angle * Space.speed
  (** [Move (a, v)] est l'angle et la vitesse souhaités pour la
     prochaine unité de temps. La vitesse ne peut pas être négative et
     elle ne peut excéder la vitesse communiquée par le serveur. *)

  | Put of microcode * Space.duration
  (** [Put (microcode, duration)] pose un [microcode] à la position courante
      du robot. Ce microcode s'autodétruira au bout de [duration] unité de
      temps. Par contre, s'il se trouve à une distance inférieure à
      [Space.small_distance] d'un microcode similaire, il est fusionné
      avec ce dernier et la durée de vie du microcode résultant est
      la somme des durées de vide des deux microcodes. *)

  | ChopTree
  (** [ChopTree] coupe une branche d'un arbre de Böhm situé une distance
      inférieure à [Space.small_distance] du robot. Cela augmente le score
      de 1 point. *)

  | Wait
  (** [Wait] ne change rien jusqu'au prochain appel. *)

  | Die of string
  (** [Die] est produit par les robots dont on a perdu le signal. *)

[@@deriving yojson]

(**

   Le problème principal de ce projet est le calcul de chemin.

   On se dote donc d'un type pour décrire un chemin : c'est une
   liste de positions dont la première est la source du chemin
   et la dernière est sa cible.

*)
type path = Space.position list

(** Version lisible des chemins. *)
let string_of_path path =
  String.concat " " (List.map string_of_position path)

(**

   Nous vous proposons de structurer le comportement du robot
   à l'aide d'objectifs décrits par le type suivant :

*)
type objective =
  | Initializing            (** Le robot doit s'initialiser.       *)
  | Chopping                (** Le robot doit couper des branches. *)
  | GoingTo of path * path
  (** Le robot suit un chemin. Le premier chemin est la liste des
      positions restantes tandis que le second est le chemin initial.
      On a donc que le premier chemin est un suffixe du second. *)

(** Version affichable des objectifs. *)
let string_of_objective = function
  | Initializing -> "initializing"
  | Chopping -> "chopping"
  | GoingTo (path, _) ->
     Printf.sprintf
       "going to %s" (String.concat " " (List.map string_of_position path))

(**

  Comme dit en introduction, le robot a une mémoire qui lui permet de
   stocker des informations sur le monde et ses actions courantes.

  On vous propose de structurer la mémoire comme suit:

*)
type memory = {
    known_world : World.t option;      (** Le monde connu par le robot.     *)
    graph       : Graph.t;             (** Un graphe qui sert de carte.     *)
    objective   : objective;           (** L'objectif courant du robot.     *)
    targets     : Space.position list; (** Les points où il doit se rendre. *)
    new_hell    : bool;
}

(**

   Initialement, le robot ne sait rien sur le monde, n'a aucune cible
   et doit s'initialiser.

*)
let initial_memory = {
    known_world = None;
    graph       = Graph.empty;
    objective   = Initializing;
    targets     = [];
    new_hell    =false;
}

(**

   Traditionnellement, la fonction de prise de décision d'un robot
   est la composée de trois fonctions :

   1. "discover" qui agrège les observations avec les observations
      déjà faites dans le passé.

   2. "plan" qui met à jour le plan courant du robot en réaction
      aux nouvelles observations.

   3. "next_action" qui décide qu'elle est l'action à effectuer
       immédiatement pour suivre le plan.

*)

(** [discover] met à jour [memory] en prenant en compte les nouvelles
    observations. *)
let discover visualize observation memory =
  let seen_world = World.world_of_observation observation in
  let known_world =
    match memory.known_world with
    | None -> seen_world
    | Some known_world -> World.extends_world known_world seen_world
  in
  if visualize then (Visualizer.show ~force:true known_world; Visualizer.show_graph memory.graph);
  { memory with known_world = Some known_world; new_hell = memory.known_world <> Some known_world }


(**

   Pour naviguer dans le monde, le robot construit une carte sous la
   forme d'un graphe.

   Les noeuds de ce graphe sont des positions clées
   du monde. Pour commencer, vous pouvez y mettre les sommets des
   polygones de l'enfer, le vaisseau, le robot et les arbres.

   Deux noeuds sont reliés par une arête si le segment dont ils
   sont les extremités ne croisent pas une bouche de l'enfer.

*)
let visibility_graph observation memory =
   {memory with graph = 
   match memory.known_world with
   | Some known_world ->
      (let hell_polygons = hell known_world in
      let hell_sommets = List.flatten (
         List.map
         (fun p->
            let ((x_pos1, y_pos1),(x_pos2, y_pos2)) = bounding_box_of_polygon p in
            Space.vertices (square (((x_pos2-.x_pos1)/.2.+.x_pos1),(((y_pos1-.y_pos2)/.2.+.y_pos2))) (x_pos2 -. x_pos1 +. 2.)())
         )
         (hell known_world)) in
      let sommets = ref (observation.spaceship :: (tree_positions (List.filter (fun tree -> tree.branches>0) observation.trees)) @ hell_sommets) in
      if List.exists ((=)observation.position) !sommets 
      then ()
      else sommets := (observation.position :: !sommets);
      let sommets = !sommets in
      Graph.make sommets
      (List.flatten (List.map
      (fun sommet1 ->
         let b = ref false in
         List.filter_map
         (fun sommet2 ->
            let segment = (sommet1, sommet2) in
            if !b || sommet1 = sommet2
            then (b:=true; None)
            else
               if List.for_all
               (fun hell_polygon ->
                  let i = ref 0 in
                  List.for_all
                  (fun hell_segment ->
                     let (hell1, hell2) = hell_segment in
                     not (segment_intersects segment hell_segment) || hell_segment = segment || if !i<2 then (i:=!i+1;hell1 = sommet1 || hell1 = sommet2 || hell2 = sommet1 || hell2 = sommet2) else false
                  )
                  (polygon_segments hell_polygon)
               )
               hell_polygons
               then let Distance d = dist2 sommet1 sommet2 in
               Some (sommet1, sommet2, d)
               else 
                  Some (sommet1, sommet2, Float.infinity)
         )
         sommets
      )
      sommets
      )))
   | None -> Graph.empty}


(**

   Il nous suffit maintenant de trouver le chemin le plus rapide pour
   aller d'une source à une cible dans le graphe.

*)
let shortest_path graph source target trees : path =
   let sort (a,b) = if a<b then (a,b) else (b,a) in
   let n = List.length (Graph.edges graph)/2 in
   let w = Hashtbl.create n in
   List.iter
   (fun u ->
      let (i,j,_) = u in
      Hashtbl.replace w (sort (i,j)) (let (_,_,poids) = u in (poids, None))
   )
   (Graph.edges graph);
   List.iter
   (fun k ->
      List.iter
      (fun segment ->
         let (i,j,_) = segment in
         if Hashtbl.find_opt w (sort (i,k)) <> None && Hashtbl.find_opt w (sort (k,j)) <> None
         then
            (let (ij,_) = (Hashtbl.find w (sort (i,j))) in
            let (ik,_) = Hashtbl.find w (sort (i,k)) in
            let (kj,_) = Hashtbl.find w (sort (k,j)) in
            if ik+.kj<ij
            then Hashtbl.replace w (sort (i,j)) (ik+.kj, Some k)
            else ())
         else ()
      )
      (Graph.edges graph)
   )
   (Graph.nodes graph);
   let rec construction_path source2 =
      let min = ref Float.infinity in
      let min_sommet = ref None in
      Hashtbl.iter
      (fun segment y ->
         let (d, deviation) = y in
         let (sommet1, sommet2) = segment in
         let sommet_mem = ref sommet2 in
         if (sommet1 = source2 || (sommet_mem:=sommet1; sommet2 = source2)) && !min > d && (tree_at trees !sommet_mem <> None)
         then (
            min_sommet := Some !sommet_mem;
            min := d;)
         else ())
      w;
      if !min_sommet = None
      then 
         Hashtbl.iter
         (fun segment y ->
            let (d, deviation) = y in
            let (sommet1, sommet2) = segment in
            let sommet_mem = ref sommet2 in
            if (sommet1 = source2 || (sommet_mem:=sommet1; sommet2 = source2)) && !sommet_mem = target
            then (
               min_sommet := Some !sommet_mem;)
            else ())
         w;
      let rec boucle_deviation sommet1 sommet2 =
         (let (_, deviation) = Hashtbl.find w (sort(sommet1,sommet2)) in
         match deviation with
         | None -> [sommet2]
         | Some dev -> (boucle_deviation sommet1 dev) @ boucle_deviation dev sommet2) in
      match !min_sommet with
      | None -> []
      | Some sommet ->
         let reponse = (boucle_deviation source2 sommet) in
         if tree_at trees source2 <> None
         then
            Hashtbl.filter_map_inplace
            (fun segment y ->
               let (sommet1, sommet2) = segment in
               let sommet_mem = ref sommet2 in
               if (sommet1 = source2 || (sommet_mem:=sommet1; sommet2 = source2)) && tree_at trees !sommet_mem <> None
               then None
               else Some y
            )
            w;
         reponse @ if sommet <> target then construction_path sommet else [] in
   construction_path source


(**

   [plan] doit mettre à jour la mémoire en fonction de l'objectif
   courant du robot.

   Si le robot est en train de récolter du bois, il n'y a rien à faire
   qu'attendre qu'il est fini.

   Si le robot est en train de suivre un chemin, il faut vérifier que
   ce chemin est encore valide compte tenu des nouvelles observations
   faites, et le recalculer si jamais ce n'est pas le cas.

   Si le robot est en phase d'initialisation, il faut fixer ses cibles
   et le faire suivre un premier chemin.

*)
let plan visualize observation memory =
   Visualizer.show_graph memory.graph;
   (match memory.objective with
   | Initializing ->
      let trees_with_branches = List.filter (fun tree -> tree.branches>0) observation.trees in
      { memory with targets = (tree_positions trees_with_branches)@[observation.spaceship]; objective = GoingTo ([List.hd (tree_positions trees_with_branches)], [List.hd (tree_positions trees_with_branches)]) }
   | Chopping ->
      (match tree_at observation.trees observation.position with
      | Some tree ->
         if tree.branches>0
         then memory
         else { memory with targets = List.tl memory.targets; objective = GoingTo ([List.hd memory.targets], [List.hd memory.targets]) }
      | None -> { memory with objective = GoingTo ([List.hd memory.targets], [List.hd memory.targets])  })
   | GoingTo (path, path2) ->
      if close (List.hd path) observation.position 1.
      then 
         (if List.length path <= 1
         then { memory with objective = Chopping }
         else { memory with objective = GoingTo (List.tl path, path2) })
      else memory)

(**

   Next action doit choisir quelle action effectuer immédiatement en
   fonction de l'objectif courant.

   Si l'objectif est de s'initialiser, la plannification a mal fait
   son travail car c'est son rôle d'initialiser le robot et de lui
   donner un nouvel objectif.

   Si l'objectif est de couper du bois, coupons du bois! Si on vient
   de couper la dernière branche, alors il faut changer d'objectif
   pour se déplacer vers une autre cible.

   Si l'objectif est de suivre un chemin, il faut s'assurer que
   la vitesse et la direction du robot sont correctes.

*)
let next_action visualize observation memory =
   match memory.objective with
   | Initializing -> Move (Space.angle_of_float 0., observation.max_speed), memory
   | Chopping -> ChopTree, memory
   | GoingTo (path, _) -> (match path with 
      | pos::_ -> 
   let Distance d = dist2 pos observation.position in
         Move (Space.angle_of_float (atan2 (y_ pos -. y_ observation.position) (x_ pos -. x_ observation.position)), if d>5. then observation.max_speed else Space.speed_of_float 0.1 ), memory
      | [] -> Move (Space.angle_of_float 0., observation.max_speed), memory)

(**

   Comme promis, la fonction de décision est la composition
   des trois fonctions du dessus.

*)
let decide visualize observation memory : action * memory =
  let memory = discover visualize observation memory in
  let memory = visibility_graph observation memory in
  let memory = plan visualize observation memory in
  next_action visualize observation memory
