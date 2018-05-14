"""
This file defines the combination of behaviors the robots will assume for each condition/run
"""
from enum_state import gen_enum_class
ProfileBase = gen_enum_class("BehaviorProfile", "disabled", "easy", "help", "dead")
class BehaviorProfile(ProfileBase):
    """ Experiment Robot Behavior Profile """
    __settings = {  
            # CONDITION 1
            (1,1,"dirtdog"):    ProfileBase.EASY,
            (1,1,"eva"):        ProfileBase.EASY,
            (1,1,"discovery"):  ProfileBase.DEAD,
            (1,1,"bender"):     ProfileBase.HELP,
            (1,1,"roomba500"):  ProfileBase.DISABLED,
            (1,1,"neato"):      ProfileBase.DISABLED,

            (1,2,"dirtdog"):    ProfileBase.HELP,
            (1,2,"eva"):        ProfileBase.HELP,
            (1,2,"discovery"):  ProfileBase.DISABLED,
            (1,2,"bender"):     ProfileBase.DISABLED,
            (1,2,"roomba500"):  ProfileBase.EASY,
            (1,2,"neato"):      ProfileBase.DEAD,


            # CONDITION 2
            (2,1,"dirtdog"):    ProfileBase.EASY,
            (2,1,"eva"):        ProfileBase.EASY,
            (2,1,"discovery"):  ProfileBase.DISABLED,
            (2,1,"bender"):     ProfileBase.DISABLED,
            (2,1,"roomba500"):  ProfileBase.HELP,
            (2,1,"neato"):      ProfileBase.DEAD,

            (2,2,"dirtdog"):    ProfileBase.HELP,
            (2,2,"eva"):        ProfileBase.HELP,
            (2,2,"discovery"):  ProfileBase.DEAD,
            (2,2,"bender"):     ProfileBase.EASY,
            (2,2,"roomba500"):  ProfileBase.DISABLED,
            (2,2,"neato"):      ProfileBase.DISABLED,


            # CONDITION 3
            (3,1,"dirtdog"):    ProfileBase.EASY,
            (3,1,"eva"):        ProfileBase.EASY,
            (3,1,"discovery"):  ProfileBase.DEAD,
            (3,1,"bender"):     ProfileBase.HELP,
            (3,1,"roomba500"):  ProfileBase.DISABLED,
            (3,1,"neato"):      ProfileBase.DISABLED,

            (3,2,"dirtdog"):    ProfileBase.HELP,
            (3,2,"eva"):        ProfileBase.HELP,
            (3,2,"discovery"):  ProfileBase.DISABLED,
            (3,2,"bender"):     ProfileBase.DISABLED,
            (3,2,"roomba500"):  ProfileBase.EASY,
            (3,2,"neato"):      ProfileBase.DEAD,


            # CONDITION 4
            (4,1,"dirtdog"):    ProfileBase.EASY,
            (4,1,"eva"):        ProfileBase.EASY,
            (4,1,"discovery"):  ProfileBase.DISABLED,
            (4,1,"bender"):     ProfileBase.DISABLED,
            (4,1,"roomba500"):  ProfileBase.HELP,
            (4,1,"neato"):      ProfileBase.DEAD,

            (4,2,"dirtdog"):    ProfileBase.HELP,
            (4,2,"eva"):        ProfileBase.HELP,
            (4,2,"discovery"):  ProfileBase.DEAD,
            (4,2,"bender"):     ProfileBase.EASY,
            (4,2,"roomba500"):  ProfileBase.DISABLED,
            (4,2,"neato"):      ProfileBase.DISABLED,

            }

    def get_robot_config(self, robot, cond_id, run_id):
        """ Internally sets, and also returns the string value, of the behavior profile for this robot """
        self.set(self.__settings[(cond_id,run_id,robot)])
        return self.get()


