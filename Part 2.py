class CourseRegistrationSystem:
    def __init__(self):
        # The dictionary acts as our Hash Table
        self.courses = {} 

    def add_course(self, course_id, capacity):
        """Adds a new course to the system."""
        self.courses[course_id] = {
            'capacity': capacity,
            'enrolled_students': set() # Using a set for O(1) student lookups
        }
        print(f"Course {course_id} added with capacity {capacity}.")

    def register_student(self, course_id, student_id):
        """Registers a student if the course exists and has space."""
        if course_id not in self.courses:
            return "Course not found."
            
        course = self.courses[course_id]
        if len(course['enrolled_students']) >= course['capacity']:
            return "Course is full."
            
        if student_id in course['enrolled_students']:
            return "Student already registered."
            
        course['enrolled_students'].add(student_id)
        return f"Student {student_id} successfully registered for {course_id}."

    def withdraw_student(self, course_id, student_id):
        """Removes a student from a course."""
        if course_id in self.courses and student_id in self.courses[course_id]['enrolled_students']:
            self.courses[course_id]['enrolled_students'].remove(student_id)
            return f"Student {student_id} withdrawn from {course_id}."
        return "Student or Course not found."

    def check_available_slots(self, course_id):
        """Returns the number of available slots in a course."""
        if course_id in self.courses:
            course = self.courses[course_id]
            available = course['capacity'] - len(course['enrolled_students'])
            return f"{course_id} has {available} slots available."
        return "Course not found."

# --- Test Cases ---
if __name__ == "__main__":
    system = CourseRegistrationSystem()
    system.add_course("PETR2133", 30)
    print(system.register_student("PETR2133", "Student_1"))
    print(system.register_student("PETR2133", "Student_1")) # Test duplicate
    print(system.check_available_slots("PETR2133"))
    print(system.withdraw_student("PETR2133", "Student_1"))
    print(system.check_available_slots("PETR2133"))