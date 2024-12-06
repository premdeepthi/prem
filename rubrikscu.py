#2*2 RUBRIK CUBE

#For our Rubik's cube we have 24 faces and for each of them we have given
#and assigned a  number from 0 to 23 faces(ex- pyb = flu =0)

# variables pyb are assigned based on the sides(front, left,up, down,down)
#and position

#Logic facing front side

#(red(r)-front(f), green(g)- left(l), white(w) - up(u))

rgw = flu = 0 # (0-th cubie; front face)
gwr = luf = 1 # (0-th cubie; left face)
wrg = ufl = 2 # (0-th cubie; up face)

#(red(r)-front(f), white(w) - up(u)), blue(b) - right(r))

rwb = fur = 3 # (1-st cubie; front face)
wbr = urf = 4 # (1-st cubie; up face)
brw = rfu = 5 # (1-st cubie; right face)

#(red(r)-front(f), green(g)- left(l),  yellow(y)- down(d))

ryg = fdl = 6 # (2-nd cubie; front face)
ygr = dlf = 7 # (2-nd cubie; down face)
gry = lfd = 8 # (2-nd cubie; left face)

#(red(r)-front(f), blue(b) - right(r),  yellow(y)- down(d))

rby = frd = 9 #  (3-rd cubie; front face)
byr = rdf = 10 # (3-rd cubie; right face)
yrb = dfr = 11 # (3-rd cubie; down face)

#Logic facing from back side

#(orange(o)- back(b) , green(g)- left(l), white(w) - up(u))

owg = bul = 12 # (4-th cubie; back face)
wgo = ulb = 13 # (4-th cubie; up face)
gow = lbu = 14 # (4-th cubie; left face)

#(orange(o)- back(b) ,white(w) - up(u)), blue(b) - right(r))

obw = bru = 15 # (5-th cubie; back face)
bwo = rub = 16 # (5-th cubie; right face)
wob = ubr = 17 # (5-th cubie; up face)

#(orange(o)- back(b), green(g)- left(l),  yellow(y)- down(d))

ogy = bld = 18 # (6-th cubie; back face)
gyo = ldb = 19 # (6-th cubie; left face)
yog = dbl = 20 # (6-th cubie; down face)

#(orange(o)- back(b), blue(b) - right(r), yellow(y)- down(d))

oyb = bdr = 21 # (7-th cubie; back face)
ybo = drb = 22 # (7-th cubie; down face)
boy = rbd = 23 # (7-th cubie; right face)

# perm_apply:
# This function applies a given permutation (`perm`) to reorder the elements of the `location` (current state).
# Each permutation represents a cube twist (move), altering the positions of the facelets.

def perm_apply(perm, location):
    # Initialize an empty list to store the reordered elements of 'location'
    result = []

    # Iterate through each index in the 'perm' list
    for i in perm:
        # Append the element at index 'i' in 'location' to the 'result' list
        result.append(location[i])

    # Convert the 'result' list into a tuple and return it
    # This represents the new state after applying the permutation
    return tuple(result)

# will return inverse permutation  of the newState    

def perm_inverse(p):
    """
    Compute the inverse of a permutation.
        p (list or tuple): A permutation represented as a sequence of indices, 
                           where each index specifies the new position of the corresponding element.
    """
    # n: The length of the permutation 'p'
    n = len(p)

    # q: Initialize a list of zeros with the same length as 'p'
    #    'q' will store the inverse permutation, where q[i] gives the original position of the element  that is at position i in the permutation 'p'.
    q = [0] * n

    # For each index i in the permutation 'p'
    for i in range(n):
        # Set the value at the position indicated by p[i] to be the index 'i'. This effectively reverses the mapping defined by 'p'.
        q[p[i]] = i

    #Returns: tuple: The inverse permutation, such that applying it to the result of `perm_apply`restores the original order.
    return tuple(q)


# perm_to_string:
# Converts a permutation into a compact string representation for easier visualization.  

def perm_to_string(p):
    """
    Convert p to string, slightly more compact
    than list printing.
    """
    s = "("
    for x in p: s = s + "%2d "%x
    s += ")"
    return s

# Identity: equal to (0, 1, 2, ..., 23).
I = (flu, luf, ufl, fur, urf, rfu, fdl, dlf, lfd, frd, rdf, dfr,bul, ulb, lbu, bru, rub, ubr, bld, ldb, dbl, bdr, drb, rbd)

"""
When any of the following Rubik's cube permutations are applied, the three faces on a cubie naturally stay together:
{0,1,2}, {3,4,5}, ..., {21,22,23}.
"""
# Move Definitions:
# Each move is a permutation that rearranges the cube's facelets.

# Front face rotated clockwise.
F = (fdl, dlf, lfd, flu, luf, ufl, frd, rdf, dfr, fur, urf, rfu, bul, ulb, lbu, bru, rub, ubr, bld, ldb, dbl, bdr, drb, rbd)
# Front face rotated counter-clockwise.
Fi = perm_inverse(F) # frontInverse

# Left face rotated clockwise.
L = (ulb, lbu, bul, fur, urf, rfu, ufl, flu, luf, frd, rdf, dfr,dbl, bld, ldb, bru, rub, ubr, dlf, lfd, fdl, bdr, drb, rbd)
# Left face rotated counter-clockwise.
Li = perm_inverse(L) #LeftInverse

# Upper face rotated clockwise.
U = (rfu, fur, urf, rub, ubr, bru, fdl, dlf, lfd, frd, rdf, dfr,luf, ufl, flu, lbu, bul, ulb, bld, ldb, dbl, bdr, drb, rbd)
# Upper face rotated counter-clockwise.
Ui = perm_inverse(U)  #UpInverse

# All 6 possible moves (assuming that the lower-bottom-right cubie
# stays fixed).
quarter_twists = (F, Fi, L, Li, U, Ui)

# Mapping each permutation to its descriptive name for better readability.
#create a varaiable to save moves
quarter_twists_names = {}
quarter_twists_names[F] = 'F'
quarter_twists_names[Fi] = 'Fi'
quarter_twists_names[L] = 'L'
quarter_twists_names[Li] = 'Li'
quarter_twists_names[U] = 'U'
quarter_twists_names[Ui] = 'Ui'