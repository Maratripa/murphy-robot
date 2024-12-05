export default defineEventHandler(async (event) => {
    console.log("Llamaste a testArea")
    await $fetch('http://localhost:6969', {
        method: 'POST',
        body: {
            command: 'testArea'
        }
    })
    return "Llamaste a testArea"
})
