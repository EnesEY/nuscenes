import load000setup
import load001InitialMovingState 

# loads the attribute movedBefore into every sample_annotation
def loadMovedBefore():
    print('start adding moved before annotation')
    # load moving state first
    load001InitialMovingState.induceMovingStateInSampleAnnotation()
    for instance in load000setup.nusc.instance:
        # information weather this instance has moved or not
        movedBefore = False
        # first and last annotation token of instance
        first_annotation_token = instance['first_annotation_token']
        last_annotation_token = instance['last_annotation_token']
        # first annotation of instance
        first_annotation = load000setup.nusc.get('sample_annotation', first_annotation_token)
        # next annotation token from first_annotation
        next_annotation_token = first_annotation['next']

        movedBefore = updateMovedBefore(movedBefore, first_annotation)
        while next_annotation_token != last_annotation_token and next_annotation_token != '':
            current_annotation = load000setup.nusc.get('sample_annotation', next_annotation_token)
            movedBefore = updateMovedBefore(movedBefore, current_annotation)
            current_annotation['movedBefore'] = movedBefore
            next_annotation_token = current_annotation['next']
    print('finished loading moved before annotation')

# gets the current moved before state and the annotation
# looks if the moved before state has changed and inputs movedBefore into the annotation also
def updateMovedBefore(movedBefore, annotation):
    if movedBefore == True:
        movedBefore = True
    if movedBefore == False and annotation['movingState'] == True:
        movedBefore = True
    return movedBefore

loadMovedBefore()
