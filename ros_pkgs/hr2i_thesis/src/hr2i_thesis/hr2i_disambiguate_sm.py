#!/usr/bin/env python
import math

from smach import StateMachine, CBState
from nao_smach_utils.tts_state import SpeechFromPoolSM
from nao_smach_utils.get_user_speech_answer import GetUserAnswer
from hr2i_thesis.msg import PointCloudClusterCentroids


class DisambiguateBlobs(StateMachine):
    ''' It assumes max 3 blobs has been detected '''
    DIST_TH = 0.1  # metres
    SIZE_TH = 0.95  # If ratio between sizemax/sizemin > 0.95 then we assume ambiguous size

    position_pool = ['Is it the one at my %s-hand side?', 'Do you mean the %s-most object?', 'Is this one at my %s, is it?',
                     'I think you were pointing to the %s-most one, were you?']

    size_pool = ['Is it the %sest one?', 'You were pointing at the %sest object, right?',
                 'Oh, do you refer to this %s thing over here?', 'Are you pointing to the %sest object?']

    def __init__(self):
        StateMachine.__init__(self, outcomes=['succeeded'], input_keys=['info_clusters', 'ground_point'],
                              output_keys=['selected_cluster_centroid'])

        with self:

            StateMachine.add('GENERATE_PREPARE_INFO', CBState(self.check_generate_cb, outcomes=['succeeded', 'disambiguate'],
                                                              input_keys=['in_cluster_info', 'in_ground_point'],
                                                              output_keys=['out_sorted_info', 'out_used_metric', 'out_asked_id', 'question_pool', 'selected_cluster_centroid']),
                             remapping={'question_pool': 'speech_pool', 'out_sorted_info': 'sorted_info',
                                        'out_used_metric': 'used_metric', 'out_asked_id': 'asked_id',
                                        'in_cluster_info': 'info_clusters', 'in_ground_point': 'ground_point'},
                             transitions={'disambiguate': 'ASK_QUESTION', 'succeeded': 'SAY_FOUND'})

            StateMachine.add('ASK_QUESTION', SpeechFromPoolSM(), remapping={'pool': 'speech_pool'},
                             transitions={'succeeded': 'LISTEN_USER', 'preempted': 'LISTEN_USER', 'aborted': 'LISTEN_USER'})

            StateMachine.add('LISTEN_USER', GetUserAnswer(get_one=True, ask_for_repetition=True),
                             remapping={'recognition_result': 'user_answer'},
                             transitions={'succeeded': 'CHECK_RECOGNITION', 'preempted': 'LISTEN_USER', 'aborted': 'LISTEN_USER'})

            def check_yesno(ud):
                if ud.user_answer == 'yes':  # We have it!
                    ud.selected_cluster_centroid = ud.sorted_info[ud.in_asked_id]
                    ud.out_speech_pool = ['I found it!', 'Of course it is!', 'Yes, I knew it.', 'Yiiii!']
                    return 'answer_yes'
                # Answer has been 'NO'
                nel = len(ud.sorted_info.cluster_sizes)
                if (nel == 2 or ud.in_asked_id == 2):  # If we only have 2 objects or the last asked object is the one at the other side (the right or the smalles one)
                    text_pool = ['Then it was the remaining one!']
                    if ud.used_metric == 'size':
                        text_pool += ['Oh so it is the medium sized one!', 'I have to check my glasses, it is the medium one']
                    else:
                        text_pool += ['So it is the one in the middle!', 'I did not see correctly that it was the object in the middle!']
                    ud.out_speech_pool = text_pool
                    ud.selected_cluster_centroid = ud.sorted_info[1]  # It will be the one at pos 1 either if there were 2 or 3 objects
                    return 'remaining_one'
                else:  # We had 3 obects and already asked one
                    ud.out_asked_id = 2
                    if ud.used_metric == 'size':
                        ud.out_speech_pool = map(lambda s: s % 'small', self.size_pool)
                    else:
                        ud.out_speech_pool = map(lambda s: s % 'right', self.position_pool)
                return 'answer_no'
            StateMachine.add('CHECK_RECOGNITION', CBState(check_yesno, outcomes=['answer_yes', 'answer_no', 'remaining_one'],
                                                          input_keys=['user_answer', 'used_metric', 'sorted_info', 'in_asked_id'],
                                                          output_keys=['selected_cluster_centroid', 'out_asked_id', 'out_speech_pool']),
                             remapping={'in_asked_id': 'asked_id', 'out_asked_id': 'asked_id', 'out_speech_pool': 'speech_pool'},
                             transitions={'answer_yes': 'SAY_FOUND',
                                          'answer_no': 'ASK_QUESTION',
                                          'remaining_one': 'SAY_FOUND'})

            StateMachine.add('SAY_FOUND', SpeechFromPoolSM(), remapping={'pool': 'speech_pool'},
                             transitions={'succeeded': 'succeeded', 'preempted': 'succeeded', 'aborted': 'succeeded'})

    def check_generate_cb(self, ud):
        ud.out_asked_id = 0  # We will always ask for the first one
        # First check if distance ratio is enough to differenciate...
        unziped_position_sorted = zip(*sorted(zip(ud.in_cluster_info.cluster_sizes, ud.in_cluster_info.cluster_centroids), reverse=True, key=lambda cinfo: cinfo[1].y))
        # We have sorted the blobs by y coordinate in descending order, so the first one will be the left-most one
        centroids_sorted = unziped_position_sorted[1]

        # Now sort by size
        unziped_size_sorted = zip(*sorted(zip(ud.in_cluster_info.cluster_sizes, ud.in_cluster_info.cluster_centroids), reverse=True))

        nel = len(ud.in_cluster_info.cluster_sizes)  # Number of elements/custers
        dists = [0]*nel  # Allocate space
        for i in xrange(0, nel):  # Compute distance between every object and the ground point
            dists[i] = math.sqrt((centroids_sorted[i].x-ud.in_ground_point.x)**2 +
                                 (centroids_sorted[i].y-ud.in_ground_point.y)**2)
        # Now check if an object is clearly pointed
        min_index, min_value = min(enumerate(dists), key=lambda val: val[1])  # Get the one with min distance to the ground point
        for i in xrange(0, nel):
            if i == min_index:
                continue
            if abs(dists[min_index]-dists[i]) < self.DIST_TH:  # The distance is more or less equal, so there's ambiguity
                break
        else:  # No break, so min_index is the closest one and the others are clearly further away
            ud.selected_cluster_centroid = centroids_sorted[min_index]
            position = 'left' if min_index == 0 else ('right' if min_index == 2 else 'middle')

            ud.question_pool = ['I see you pointed at the in the %s.' % position,
                                'Oh the one in the %s.' % position,
                                'Yes, you were pointing at the one in the %s.' % position,
                                'I think that you meant the object in the %s.' % position]
            return 'succeeded'

        # Check if size is not ambiguous and use size if it isn't
        pcc_sorted = PointCloudClusterCentroids()
        pcc_sorted.cluster_sizes = unziped_size_sorted[0]
        pcc_sorted.cluster_centroids = unziped_size_sorted[1]
        ambiguous = False
        for i in xrange(0, nel):
            for j in xrange(i+1, nel):
                if (pcc_sorted.cluster_sizes[j]/pcc_sorted.cluster_sizes[i]) > self.SIZE_TH:
                    # They are sorted in descending order, so [j]/[i] will be always below 1.
                    # If this happens, we have ambiguity between sizes
                    ambiguous = True
                    break
            if ambiguous:
                break
        if not ambiguous:
            ud.out_sorted_info = pcc_sorted
            ud.out_used_metric = 'size'
            ud.question_pool = map(lambda s: s % 'bigg', self.size_pool)
            return 'disambiguate'

        # We have ambiguity, let's check spatial relationship
        pcc_sorted.cluster_sizes = unziped_position_sorted[0]
        pcc_sorted.cluster_centroids = unziped_position_sorted[1]
        ud.out_sorted_info = pcc_sorted
        ud.out_used_metric = 'position'
        ud.question_pool = map(lambda s: s % 'left', self.position_pool)
        return 'disambiguate'
