#
# Pseudocode for the VRP-DP algorithm, to orden my thoughts a bit.
# Using python syntax for the sake of syntax highlighting. Not working code.
# The TSP-DP algorithm is strucutred similarly.
#
def vrpDP():
    return vrpTable(X0, "2*TRUCKS, 2, 2, ..., 2 | {0, 0, CAPACITY} * TRUCKS")


def vrpTable(Xi, S):
    # Look up a value in a table, or calculate it if it doesn't exist yet.
    if hashlist.contains(S):
        return hashlist[S]
    else:
        D, M = parse(S)
        value = vrpRecurse(Xi, D, M, [])
        hashlist[S] = value
        for unused_demand in range(C - value.used_demand): # Optimization: Not yet in the code - TODO
            hashlis[S - unused_demand] = value
        return value


#    ,,,
#   ('v')
#  <('"')>
#    " "
def vrpRecurse(Xi, D, M, loopvars):
    # Calculate the allowed entries for the table (degrees, matchings) (and the 'leftovers' for the children).
    if final_base_case(loopvars):
        return vrpChildEvaluation(Xi, D, M, loopvars)
    if uninteresting_base_cases(loopvars):
        return vrpRecurse(Xi, D, M, next(loopvars))

    if D >= 2 and isDepot(loopvars):
        for e in range(depot_to_depot_edges(M)):
            vrpRecurse(Xi, D-2, M, next(loopvars + 2 + {0, 0}))
    if D >= 2 and not isDepot(loopvars):
        vrpRecurse(Xi, D-2, M, next(loopvars+2))
    if D >= 1:
        for k in range(current, end):
            D -= 1+1
            loopvars += 1+1
            loopvars += {current, k}
            vrpRecurse(Xi, D, M, next(loopvars))
    if D >= 0:
        vrpRecurse(Xi, D, M, next(loopvars))
    return bestResultOfAllRecurses


def vrpChildEvaluation(Xi, D, M, childDs, childMs):
    # This function is basically the base case of the recurse function.
    if obviousCycle(M) and Xi != Xroot:
        return infinity

    possibleEdgelists = vrpEdgeSelect(sorted(Xi.edgeSet), D+M, childDs+childMs)

    resultValue = infinity
    for edgeValue, edgeList, demands in possibleEdgelists:
        if isLeafBag(Xi):
            return edgeValue
        possibleMatchings = allChildMatchings(edgeList)
        for kidM in possibleMatchings:
            value = edgeValue
            for Xj, kidD in zip(Xi.children, childDs):
                adjust(kidD)
                value += vrpTable(Xj, parse(kidD, kidM))
            resultValue = min(resultValue, value)
    return resultValue


def vrpEdgeSelect(Yi, target, childTargets, loopvars):
    # Calculate the tour-edges used in this bag.
    if satisfied(target):
        # The cycle check contains the TSP exception for the root bag (not interesting for VRP, but good to know anyway).
        # For the VRP this also contains the check if the paths don't exceed the demand limit.
        if not cycleCheck(loopvars.edges, target.M, childTargets.Ms):
            return []
        return (value, loopvars.edgeBits)

    if fail_base_case(target, loopvars):
        return []

    result_with = vrpEdgeSelect(Yi, target, childTargets, next(loopvars + edge))
    result_without = vrpEdgeSelect(Yi, target, childTargets, next(loopvars))
    return merge(result_with, result_without)


#   _===_
#  \(.,.)
#   ( : )\
#   ( : )
def allChildMatchings(Xi, paths):
    # This one first groups all the subpaths for all the main paths,
    # and then starts looping all possible ways of dividing the demands over them.
    # It's a rather precise method and annoying to get right, but not so important for the main overview.
    # Note that the second part is located inside an if in the first part.
    return [
        len(Xi.children) * [
            nrOfPossibilities * [
                createMatching()
            ]
        ]
    ]

