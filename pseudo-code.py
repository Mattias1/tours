#
# Pseudocode for the VRP-DP algorithm, to orden my thoughts a bit.
# Using python syntax for the sake of syntax highlighting. Not working code.
# The TSP-DP algorithm is strucutred similarly.
#
def vrpDP():
    return vrpTable(X0, "2*TRUCKS, 2, 2, ..., 2 | {0, 0, CAPACITY} * TRUCKS")


def vrpTable(Xi, S):
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
    if final_base_case(loopvars):
        return vrpChildEvaluation(Xi, D, M, loopvars)
    if uninteresting_base_case(loopvars):
        return vrpRecurse(Xi, D, M, next(loopvars))

    if isDepot(loopvars):
        for e in range(depot_to_depot_edges(M)):
            vrpRecurse(Xi, D-2, M, next(loopvars + 2 + {0, 0}))
    if D >= 2:
        if isDepot:
            vrpRecurse(Xi, D, M+{0,0}, next(loopvars+2))
        else:
            vrpRecurse(Xi, D-2, M, next(loopvars+2))
    if D >= 1:
        for k in range(current, end):
            D -= 1+1
            loopvars += 1+1
            loopvars += {current, k}
            vrpRecurse(Xi, D, M, next(loopvars))
    if D >= 0:
        vrpRecurse(Xi, D, M, next(loopvars))


def vrpChildEvaluation(Xi, D, M, childDs, childMs):
    if obviousCycle(M) and Xi != Xroot:
        return infinity

    possibleEdgelists = vrpEdgeSelect(sorted(Xi.edgeSet), D+M, childDs+childMs)

    resultValue = infinity
    for edgeValue, edgeList, demands in possibleEdgelists:
        if isLeafBag(Xi):
            if isValid(demands): # TODO: isValidDemands is not yet in the code!!! (TODO: add it in edgeSelect? (currently removed there))
                return edgeValue
            else:
                continue;
        possibleMatchings = allChildMatchings(edgeList)
        for kidM in possibleMatchings:
            value = edgeValue
            for Xj, kidD in zip(Xi.children, childDs):
                adjust(kidD)
                value += vrpTable(Xj, parse(kidD, kidM))
            resultValue = min(resultValue, value)
    return resultValue


def vrpEdgeSelect(Yi, target, childTargets, loopvars):
    if satisfied(target):
        # The cycle check contains the TSP exception for the root bag (not interesting for VRP, but good to know anyway).
        # For the VRP this also contains the check if the paths don't exceed the demand limit.
        if not cycleCheck(loopvars.edges, target.M, childTargets.Ms):
            return []
        demandMatching = pathDemands(Xi, loopvars.edges, target.M, childTargets.Ms)
        if not_valid(demandMatching) and not rootbag:
            return []
        return (value, loopvars.edgeBits, demandMatching)

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
    return [
        len(Xi.children) * [
            nrOfPossibilities * [
                createMatching()
            ]
        ]
    ]

