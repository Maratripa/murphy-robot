export default defineEventHandler(async (event) => {
    console.log("Llamaste a startRoutine")
    await $fetch('http://localhost:6969', {
        method: 'POST',
        body: {
            command: 'startRoutine'
        }
    })
    return "Llamaste a startRoutine"
})
